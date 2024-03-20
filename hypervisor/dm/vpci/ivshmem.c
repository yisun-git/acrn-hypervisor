/*-
 * Copyright (C) 2020-2022 Intel Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY NETAPP, INC ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NETAPP, INC OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifdef CONFIG_IVSHMEM_ENABLED
#include <asm/guest/vm.h>
#include <asm/mmu.h>
#include <asm/guest/ept.h>
#include <logmsg.h>
#include <errno.h>
#include <ivshmem.h>
#include <ivshmem_cfg.h>
#include "vpci_priv.h"

/**
 * @addtogroup vp-dm_vperipheral
 *
 * @{
 */

/**
 * @file
 * @brief This file declares ivshmem device functions, data structures and macros
 *
 * This file declares ivshmem device functions, data structures, and macros to handle shared memory events between VMs.
 *
 */

#define	IVSHMEM_CLASS		0x05U /**< Pre-defined class code of the ivshmem device. */
#define	IVSHMEM_REV		0x01U /**< Pre-defined revision ID of the ivshmem device. */
#define IVSHMEM_MMIO_BAR	0U /**< Pre-defined mmio bar of the ivshmem device. */
#define IVSHMEM_MSIX_BAR	1U /**< Pre-defined msix bar of the ivshmem device. */
#define IVSHMEM_SHM_BAR		2U /**< Pre-defined shared memory region bar of the ivshmem device. */
#define IVSHMEM_MMIO_BAR_SIZE	256UL /**< Pre-defined mmio bar size of the ivshmem device. */
#define	IVSHMEM_IRQ_MASK_REG	0x0U /**< Pre-defined interrupt mask register offset of the ivshmem device. */
#define	IVSHMEM_IRQ_STA_REG	0x4U /**< Pre-defined interrupt status register offset of the ivshmem device. */
#define	IVSHMEM_IV_POS_REG	0x8U /**< Pre-defined inter-vm position register offset of the ivshmem device. */
#define	IVSHMEM_DOORBELL_REG	0xcU /**< Pre-defined doorbell register offset of the ivshmem device. */

/**  The shared memory region list of ivshmem device */
static struct ivshmem_shm_region mem_regions[8] = {
	IVSHMEM_SHM_REGIONS
};

/**
 * @brief Union of ivshmem device doorbell register
 *
 * The ivshmem device doorbell register value is kept in this union.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
union ivshmem_doorbell {
	uint32_t val;
	struct {
		uint16_t vector_index;
		uint16_t peer_id;
	} reg;
};

/**
 * @brief Data structure to present ivshmem device
 *
 * The structure includes the ivshmem device information, the corresponding pci device, the ivshmem device
 * registers and the shared memory regions.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct ivshmem_device {
	struct pci_vdev* pcidev;
	union {
		uint32_t data[4];
		struct {
			uint32_t irq_mask;
			uint32_t irq_state;
			/*
			 * If the device is not configured for interrupts,
			 * this is zero. Else, ivpos is the device's ID.
			 */
			uint32_t ivpos;

			/* Writing doorbell register requests to interrupt a peer */
			union ivshmem_doorbell doorbell;
		} regs;
	} mmio;
	struct ivshmem_shm_region *region;
};

static struct ivshmem_device ivshmem_dev[IVSHMEM_DEV_NUM]; /**< Array of ivshmem device */
static spinlock_t ivshmem_dev_lock = { .head = 0U, .tail = 0U, }; /**< Spin lock used to create/deinit ivshmem device */

void init_ivshmem_shared_memory()
{
	uint32_t i;
	uint64_t addr;

	addr = e820_alloc_memory(roundup(IVSHMEM_SHM_SIZE, PDE_SIZE), MEM_SIZE_MAX);
	for (i = 0U; i < ARRAY_SIZE(mem_regions); i++) {
		mem_regions[i].hpa = addr;
		addr += mem_regions[i].size;
	}
}

/**
 * @brief Find shared memory region by name
 *
 * @param[in] name The name string used to find the shared memory region.
 *
 * @return The shared memory region found out by name. It could be NULL if no region found out.
 *
 * @pre name != NULL
 *
 * @post N/A
 *
 * @remark N/A
 */
static struct ivshmem_shm_region *find_shm_region(const char *name)
{
	uint32_t i, num = ARRAY_SIZE(mem_regions);

	for (i = 0U; i < num; i++) {
		if (strncmp(name, mem_regions[i].name, sizeof(mem_regions[0].name)) == 0) {
			break;
		}
	}
	return ((i < num) ? &mem_regions[i] : NULL);
}

/*
 * @brief Bind the ivshmem device to a shared memory region
 *
 * There are two ivshmem server implementation in HV-land and DM-land, they're used for briding the notification
 * channel between ivshmem devices acrossed VMs.
 *
 * @param[in] vdev Pointer to the virtual PCI device to access.
 *
 * @return None
 *
 * @pre vdev != NULL
 * @pre region->doorbell_peers[vm_id] = NULL
 *
 * @post region->doorbell_peers[vm_id] != NULL
 *
 * @remark N/A
 */
static void ivshmem_server_bind_peer(struct pci_vdev *vdev)
{
	uint16_t vm_id;
	struct acrn_vm_pci_dev_config *dev_config = vdev->pci_dev_config;
	struct ivshmem_device *ivs_dev = (struct ivshmem_device *)vdev->priv_data;
	struct ivshmem_shm_region *region = find_shm_region(dev_config->shm_region_name);

	if (region != NULL) {
		vm_id = vpci2vm(vdev->vpci)->vm_id;
		/* Device ID equals to VM ID*/
		ivs_dev->mmio.regs.ivpos = vm_id;
		ivs_dev->region = region;
		region->doorbell_peers[vm_id] = ivs_dev;
	}
}

/*
 * @brief Unbind the ivshmem device from a shared memory region
 *
 * @param[in] vdev Pointer to the virtual PCI device to access.
 *
 * @return None
 *
 * @pre vdev != NULL
 *
 * @post region->doorbell_peers[vpci2vm(vdev->vpci)->vm_id] == NULL
 *
 * @remark N/A
 */
static void ivshmem_server_unbind_peer(struct pci_vdev *vdev)
{
	struct ivshmem_shm_region *region = ((struct ivshmem_device *)vdev->priv_data)->region;

	region->doorbell_peers[vpci2vm(vdev->vpci)->vm_id] = NULL;
}

/*
 * @brief Inject MSI to target VM if the doorbell is triggered
 *
 * The fucntion is used to notify target VM to check shared memory when xource VM writes ivshmem
 * device doorbell register.
 *
 * @param[in] src_ivs_dev  Pointer to source ivshmem device.
 * @param[in] dest_peer_id The destination ID used to find the peer ivshmem device.
 * @param[in] vector_index The vector index that need to be injected to destination VM.
 *
 * @return None
 *
 * @pre src_ivs_dev != NULL
 *
 * @post N/A
 *
 * @remark N/A
 */
static void ivshmem_server_notify_peer(struct ivshmem_device *src_ivs_dev, uint16_t dest_peer_id, uint16_t vector_index)
{
	struct acrn_vm *dest_vm;
	struct ivshmem_device *dest_ivs_dev;
	struct msix_table_entry *entry;
	struct ivshmem_shm_region *region = src_ivs_dev->region;

	if (dest_peer_id < MAX_IVSHMEM_PEER_NUM) {

		dest_ivs_dev = region->doorbell_peers[dest_peer_id];
		if ((dest_ivs_dev != NULL) && vpci_vmsix_enabled(dest_ivs_dev->pcidev)
			&& (vector_index < dest_ivs_dev->pcidev->msix.table_count)) {

			entry = &(dest_ivs_dev->pcidev->msix.table_entries[vector_index]);
			if ((entry->vector_control & PCIM_MSIX_VCTRL_MASK) == 0U) {

				dest_vm = vpci2vm(dest_ivs_dev->pcidev->vpci);
				vlapic_inject_msi(dest_vm, entry->addr, entry->data);
			} else {
				pr_err("%s,target msix entry [%d] is masked.\n",
					__func__, vector_index);
			}
		} else {
			pr_err("%s,Invalid peer, ID = %d, vector index [%d] or MSI-X is disabled.\n",
				__func__, dest_peer_id, vector_index);
		}
	}
}

/*
 * @brief Put the input vdev into the ivshmem_dev array
 *
 * @param[in] vdev Pointer to the virtual PCI device to access.
 *
 * @return None
 *
 * @pre vdev != NULL
 *
 * @post vdev->priv_data != NULL
 *
 * @remark N/A
 */
static void create_ivshmem_device(struct pci_vdev *vdev)
{
	uint32_t i;

	spinlock_obtain(&ivshmem_dev_lock);
	for (i = 0U; i < IVSHMEM_DEV_NUM; i++) {
		if (ivshmem_dev[i].pcidev == NULL) {
			ivshmem_dev[i].pcidev = vdev;
			vdev->priv_data = &ivshmem_dev[i];
			break;
		}
	}
	spinlock_release(&ivshmem_dev_lock);
	ASSERT((i < IVSHMEM_DEV_NUM), "failed to find and set ivshmem device");
	/*
	 * Clear ivshmem_device mmio to ensure the same initial
	 * states after VM reboot.
	 */
	memset(&ivshmem_dev[i].mmio, 0U, sizeof(uint32_t) * 4);
}

/*
 * @brief Handle mmio access from VM
 *
 * When VM accesses ivshmem device mmio, it traps and is handled in this function.
 *
 * @param[in] io_req Pointer to the mmio request.
 * @param[in] data   Pointer to the virtual PCI device to access.
 *
 * @return 0
 *
 * @pre vdev->priv_data != NULL
 *
 * @post vdev->priv_data != NULL
 *
 * @remark N/A
 */
static int32_t ivshmem_mmio_handler(struct io_request *io_req, void *data)
{
	union ivshmem_doorbell doorbell;
	struct acrn_mmio_request *mmio = &io_req->reqs.mmio_request;
	struct pci_vdev *vdev = (struct pci_vdev *) data;
	struct ivshmem_device *ivs_dev = (struct ivshmem_device *) vdev->priv_data;
	uint64_t offset = mmio->address - vdev->vbars[IVSHMEM_MMIO_BAR].base_gpa;

	/* ivshmem spec define the BAR0 offset > 16 are reserved */
	if ((mmio->size == 4U) && ((offset & 0x3U) == 0U) &&
		(offset < sizeof(ivs_dev->mmio))) {
		/*
		 * IVSHMEM_IRQ_MASK_REG and IVSHMEM_IRQ_STA_REG are R/W registers
		 * they are useless for ivshmem Rev.1.
		 * IVSHMEM_IV_POS_REG is Read-Only register and IVSHMEM_DOORBELL_REG
		 * is Write-Only register, they are used for interrupt.
		 */
		if (mmio->direction == ACRN_IOREQ_DIR_READ) {
			if (offset != IVSHMEM_DOORBELL_REG) {
				mmio->value = ivs_dev->mmio.data[offset >> 2U];
			} else {
				mmio->value = 0UL;
			}
		} else {
			if (offset != IVSHMEM_IV_POS_REG) {
				if (offset == IVSHMEM_DOORBELL_REG) {
					doorbell.val = mmio->value;
					ivshmem_server_notify_peer(ivs_dev, doorbell.reg.peer_id,
						doorbell.reg.vector_index);
				} else {
					ivs_dev->mmio.data[offset >> 2U] = mmio->value;
				}
			}
		}
	}
	return 0;
}

/*
 * @brief Read ivshmem device config
 *
 * @param[in]  vdev   Pointer to the virtual PCI device to access.
 * @param[in]  offset The offset of config space to read.
 * @param[in]  bytes  The size of data to read.
 * @param[out] val    Pointer to the buffer of the value read out from config space.
 *
 * @return 0
 *
 * @pre vdev != NULL
 * @pre val != NULL
 *
 * @remark N/A
 */
static int32_t read_ivshmem_vdev_cfg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t *val)
{
	*val = pci_vdev_read_vcfg(vdev, offset, bytes);

	return 0;
}

/*
 * @brief Unmap ivshmem device bar
 *
 * This function unmaps ivshmem device bar from EPT or unregister the mmio handler.
 *
 * @param[in] vdev Pointer to the virtual PCI device to access.
 * @param[in] idx  The BAR ID to do unmap.
 *
 * @return None
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
static void ivshmem_vbar_unmap(struct pci_vdev *vdev, uint32_t idx)
{
	struct acrn_vm *vm = vpci2vm(vdev->vpci);
	struct pci_vbar *vbar = &vdev->vbars[idx];

	if ((idx == IVSHMEM_SHM_BAR) && (vbar->base_gpa != 0UL)) {
		ept_del_mr(vm, (uint64_t *)vm->arch_vm.nworld_eptp, vbar->base_gpa, vbar->size);
	} else if (((idx == IVSHMEM_MMIO_BAR) || (idx == IVSHMEM_MSIX_BAR)) && (vbar->base_gpa != 0UL)) {
		unregister_mmio_emulation_handler(vm, vbar->base_gpa, (vbar->base_gpa + vbar->size));
	}
}

/*
 * @brief Map ivshmem device bar
 *
 * This function maps ivshmem device bar to EPT or register the mmio handler and delete the bar from EPT.
 *
 * @param[in] vdev Pointer to the virtual PCI device to access.
 * @param[in] idx  The BAR ID to do map.
 *
 * @return None
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
static void ivshmem_vbar_map(struct pci_vdev *vdev, uint32_t idx)
{
	struct acrn_vm *vm = vpci2vm(vdev->vpci);
	struct pci_vbar *vbar = &vdev->vbars[idx];

	if ((idx == IVSHMEM_SHM_BAR) && (vbar->base_hpa != INVALID_HPA) && (vbar->base_gpa != 0UL)) {
		ept_add_mr(vm, (uint64_t *)vm->arch_vm.nworld_eptp, vbar->base_hpa,
				vbar->base_gpa, vbar->size, EPT_RD | EPT_WR | EPT_WB | EPT_IGNORE_PAT);
	} else if ((idx == IVSHMEM_MMIO_BAR) && (vbar->base_gpa != 0UL)) {
		register_mmio_emulation_handler(vm, ivshmem_mmio_handler, vbar->base_gpa,
				(vbar->base_gpa + vbar->size), vdev, false);
		ept_del_mr(vm, (uint64_t *)vm->arch_vm.nworld_eptp, vbar->base_gpa, round_page_up(vbar->size));
	} else if ((idx == IVSHMEM_MSIX_BAR) && (vbar->base_gpa != 0UL)) {
		register_mmio_emulation_handler(vm, vmsix_handle_table_mmio_access, vbar->base_gpa,
			(vbar->base_gpa + vbar->size), vdev, false);
		ept_del_mr(vm, (uint64_t *)vm->arch_vm.nworld_eptp, vbar->base_gpa, vbar->size);
		vdev->msix.mmio_gpa = vbar->base_gpa;
	}
}

/*
 * @brief Write ivshmem device config
 *
 * @param[in,out] vdev   Pointer to the virtual PCI device to access.
 * @param[in]     offset The offset of config space to write.
 * @param[in]     bytes  The size of data to write.
 * @param[in]     val    The value to be written into config space.
 *
 * @return 0
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
static int32_t write_ivshmem_vdev_cfg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t val)
{
	if (vbar_access(vdev, offset)) {
		vpci_update_one_vbar(vdev, pci_bar_index(offset), val,
			ivshmem_vbar_map, ivshmem_vbar_unmap);
	} else if (msixcap_access(vdev, offset)) {
		write_vmsix_cap_reg(vdev, offset, bytes, val);
	} else {
		pci_vdev_write_vcfg(vdev, offset, bytes, val);
	}

	return 0;
}

/*
 * @brief Initialize ivshmem device bar
 *
 * @param[in,out] vdev    Pointer to the virtual PCI device to access.
 * @param[in]     bar_idx The BAR ID to initialize.
 *
 * @return None
 *
 * @pre vdev != NULL
 * @pre bar_idx < PCI_BAR_COUNT
 *
 * @remark N/A
 */
static void init_ivshmem_bar(struct pci_vdev *vdev, uint32_t bar_idx)
{
	struct pci_vbar *vbar;
	uint64_t addr, mask, size = 0UL;
	struct acrn_vm_pci_dev_config *dev_config = vdev->pci_dev_config;

	addr = dev_config->vbar_base[bar_idx];
	vbar = &vdev->vbars[bar_idx];
	vbar->bar_type.bits = addr;
	mask = is_pci_io_bar(vbar) ? PCI_BASE_ADDRESS_IO_MASK : PCI_BASE_ADDRESS_MEM_MASK;
	vbar->bar_type.bits &= (~mask);

	if (bar_idx == IVSHMEM_SHM_BAR) {
		struct ivshmem_shm_region *region = find_shm_region(dev_config->shm_region_name);
		if (region != NULL) {
			size = region->size;
			vbar->base_hpa = region->hpa;
		} else {
			pr_err("%s ivshmem device %x:%x.%x has no memory region\n",
				__func__, vdev->bdf.bits.b, vdev->bdf.bits.d, vdev->bdf.bits.f);
		}
	} else if (bar_idx == IVSHMEM_MSIX_BAR) {
		size = VMSIX_ENTRY_TABLE_PBA_BAR_SIZE;
	} else if (bar_idx == IVSHMEM_MMIO_BAR) {
		size = IVSHMEM_MMIO_BAR_SIZE;
	}
	if (size != 0UL) {
		vbar->size = size;
		vbar->mask = (uint32_t) (~(size - 1UL));
		pci_vdev_write_vbar(vdev, bar_idx, (uint32_t)addr);
		if (is_pci_mem64lo_bar(vbar)) {
			vbar = &vdev->vbars[bar_idx + 1U];
			vbar->is_mem64hi = true;
			vbar->mask = (uint32_t) ((~(size - 1UL)) >> 32U);
			pci_vdev_write_vbar(vdev, (bar_idx + 1U), ((uint32_t)(addr >> 32U)));
		}
	}
}

/*
 * @brief Initialize an ivshmem device
 *
 * This function creates an ivshmem device, initialize its config space and bars, and bind it to a shared memory region.
 *
 * @param[in,out] vdev Pointer to the virtual PCI device to access.
 *
 * @return None
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
static void init_ivshmem_vdev(struct pci_vdev *vdev)
{
	create_ivshmem_device(vdev);

	/* initialize ivshmem config */
	pci_vdev_write_vcfg(vdev, PCIR_VENDOR, 2U, IVSHMEM_VENDOR_ID);
	pci_vdev_write_vcfg(vdev, PCIR_DEVICE, 2U, IVSHMEM_DEVICE_ID);
	pci_vdev_write_vcfg(vdev, PCIR_REVID, 1U, IVSHMEM_REV);
	pci_vdev_write_vcfg(vdev, PCIR_CLASS, 1U, IVSHMEM_CLASS);
	pci_vdev_write_vcfg(vdev, PCIR_HDRTYPE, 1U,
		PCIM_HDRTYPE_NORMAL | ((vdev->bdf.bits.f == 0U) ? PCIM_MFDEV : 0U));
	add_vmsix_capability(vdev, MAX_IVSHMEM_MSIX_TBL_ENTRY_NUM, IVSHMEM_MSIX_BAR);

	/* initialize ivshmem bars */
	vdev->nr_bars = 4U;
	init_ivshmem_bar(vdev, IVSHMEM_MMIO_BAR);
	init_ivshmem_bar(vdev, IVSHMEM_MSIX_BAR);
	init_ivshmem_bar(vdev, IVSHMEM_SHM_BAR);
	ivshmem_server_bind_peer(vdev);

	vdev->user = vdev;
}

/*
 * @brief Deinit an ivshmem device
 *
 * @param[in] vdev Pointer to the virtual PCI device to access.
 *
 * @return None
 *
 * @pre vdev != NULL
 * @pre vdev->priv_data != NULL
 *
 * @remark N/A
 */
static void deinit_ivshmem_vdev(struct pci_vdev *vdev)
{
	struct ivshmem_device *ivs_dev = (struct ivshmem_device *) vdev->priv_data;

	ivshmem_server_unbind_peer(vdev);

	spinlock_obtain(&ivshmem_dev_lock);
	vdev->priv_data = NULL;
	vdev->user = NULL;
	ivs_dev->pcidev = NULL;
	spinlock_release(&ivshmem_dev_lock);
}

int32_t create_ivshmem_vdev(struct acrn_vm *vm, struct acrn_vdev *dev)
{
	uint32_t i;
	struct acrn_vm_config *vm_config = get_vm_config(vm->vm_id);
	struct acrn_vm_pci_dev_config *dev_config = NULL;
	int32_t ret = -EINVAL;

	for (i = 0U; i < vm_config->pci_dev_num; i++) {
		dev_config = &vm_config->pci_devs[i];
		if (strncmp(dev_config->shm_region_name, (char *)dev->args, sizeof(dev_config->shm_region_name)) == 0) {
			struct ivshmem_shm_region *region = find_shm_region(dev_config->shm_region_name);
			if ((region != NULL) && (region->size == dev->io_size[IVSHMEM_SHM_BAR])) {
				spinlock_obtain(&vm->vpci.lock);
				dev_config->vbdf.value = (uint16_t) dev->slot;
				dev_config->vbar_base[IVSHMEM_MMIO_BAR] = (uint64_t) dev->io_addr[IVSHMEM_MMIO_BAR];
				dev_config->vbar_base[IVSHMEM_MSIX_BAR] = (uint64_t) dev->io_addr[IVSHMEM_MSIX_BAR];
				dev_config->vbar_base[IVSHMEM_SHM_BAR] = (uint64_t) dev->io_addr[IVSHMEM_SHM_BAR];
				dev_config->vbar_base[IVSHMEM_SHM_BAR] |= ((uint64_t) dev->io_addr[IVSHMEM_SHM_BAR + 1U]) << 32U;
				(void) vpci_init_vdev(&vm->vpci, dev_config, NULL);
				spinlock_release(&vm->vpci.lock);
				ret = 0;
			} else {
				pr_warn("%s, failed to create ivshmem device %x:%x.%x\n", __func__,
				dev->slot >> 8U, (dev->slot >> 3U) & 0x1fU, dev->slot & 0x7U);
			}
			break;
		}
	}
	return ret;
}

int32_t destroy_ivshmem_vdev(struct pci_vdev *vdev)
{
	uint32_t i;

	for (i = 0U; i < vdev->nr_bars; i++) {
		vpci_update_one_vbar(vdev, i, 0U, NULL, ivshmem_vbar_unmap);
	}

	return 0;
}

/**
 * @brief A set of callback functions used to operate the ivshmem device.
 */
const struct pci_vdev_ops vpci_ivshmem_ops = {
	.init_vdev	= init_ivshmem_vdev,
	.deinit_vdev	= deinit_ivshmem_vdev,
	.write_vdev_cfg	= write_ivshmem_vdev_cfg,
	.read_vdev_cfg	= read_ivshmem_vdev_cfg,
};
#endif
