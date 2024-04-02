/*-
* Copyright (c) 2011 NetApp, Inc.
* Copyright (c) 2018-2022 Intel Corporation.
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

#ifndef VPCI_H_
#define VPCI_H_

/**
 * @addtogroup vp-dm_vperipheral
 *
 * @{
 */

/**
 * @file
 *
 * @brief This file declares all external APIs that shall be provided by the virtual PCI (vPCI) component.
 *
 * This file declares all external functions, data structures, and macros that shall be provided by the vPCI
 * component in vp-dm.vperipheral module.
 *
 */

#include <asm/lib/spinlock.h>
#include <pci.h>
#include <list.h>

/**
 * @brief Pre-defined macros used for virtual device list in struct acrn_vpci.
 */
#define VDEV_LIST_HASHBITS 4U
#define VDEV_LIST_HASHSIZE (1U << VDEV_LIST_HASHBITS)

/**
 * @brief Data structure to present PCI BAR information.
 *
 * For a PCI BAR, this structure includes its type (MMIO or IO BAR and 32bits or 64bits), guest sizing
 * this BAR, BAR size, GPA/HPA of BAR base, BAR memory type (low 4bits of a BAR register) and BAR size mask.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct pci_vbar {
	bool is_mem64hi;	        /** This is to indicate the high part of 64 bits MMIO bar. */
	bool sizing;		        /** This is to indicate the guest is sizing this BAR. */
	uint64_t size;		        /** BAR size. */
	uint64_t base_gpa;	        /** BAR guest physical address. */
	uint64_t base_hpa;	        /** BAR host physical address. */
	union pci_bar_type bar_type;    /** The low 2(PIO)/4(MMIO) bits of BAR. */
	uint32_t mask;		        /** BAR size mask. */
};

/**
 * @brief Data structure to present PCI MSI-X table structure.
 *
 * For PCI MSI-X, this structure includes MSI-X table entry message address, message data
 * and its vector control.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct msix_table_entry {
	uint64_t	addr;
	uint32_t	data;
	uint32_t	vector_control;
};

/**
 * @brief Data structure to present PCI MSI capability basic information.
 *
 * For a PCI MSI capability, this structure includes its type (32bits or 64bits capability),
 * offset and length.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct pci_msi {
	bool      is_64bit;
	uint32_t  capoff;
	uint32_t  caplen;
};

/**
 * @brief Data structure to present PCI MSI-X capability structure.
 *
 * This structure presents PCI MSI-X capability structure.
 *
 * @consistency N/A
 * @alignment 1
 *
 * @remark N/A
 */
struct msixcap {
	uint8_t		capid;
	uint8_t		nextptr;
	uint16_t	msgctrl;
	uint32_t	table_info;	/** bar index and offset */
	uint32_t	pba_info;	/** bar index and offset */
} __packed;

/**
 * @brief Data structure to present PCI MSI-X capability information.
 *
 * This structure saves PCI MSI-X capability and table information.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct pci_msix {
	struct msix_table_entry table_entries[CONFIG_MAX_MSIX_TABLE_NUM];
	uint64_t  mmio_gpa;
	uint64_t  mmio_hpa;
	uint64_t  mmio_size;
	uint32_t  capoff;
	uint32_t  caplen;
	uint32_t  table_bar;
	uint32_t  table_offset;
	uint32_t  table_count;
	bool      is_vmsix_on_msi;
	bool	  is_vmsix_on_msi_programmed;
};

/**
 * @brief Data structure to present PCI SRIOV capability information.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct pci_cap_sriov {
	uint32_t  capoff;
	uint32_t  caplen;

	/**
	 * If the vdev is a SRIOV PF vdev, the vbars is used to store
	 * the bar information that is using to initialize SRIOV VF vdev bar.
	 */
	struct pci_vbar vbars[PCI_BAR_COUNT];
};

/**
 * @brief A union data structure to store all the data of a PCIe configuration space.
 *
 * It includes 4096 bytes. Also it can be accessed as 2 bytes or 4 bytes aligned.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
union pci_cfgdata {
	uint8_t data_8[PCIE_CONFIG_SPACE_SIZE];
	uint16_t data_16[PCIE_CONFIG_SPACE_SIZE >> 1U];
	uint32_t data_32[PCIE_CONFIG_SPACE_SIZE >> 2U];
};

struct pci_vdev;

/**
 * @brief Data structure to present a set of operation functions to a vPCI device.
 *
 * It includes 4 functions: init / de-init / read / write.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct pci_vdev_ops {
       void    (*init_vdev)(struct pci_vdev *vdev);
       void    (*deinit_vdev)(struct pci_vdev *vdev);
       int32_t (*write_vdev_cfg)(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t val);
       int32_t (*read_vdev_cfg)(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t *val);
};

/**
 * @brief Data structure to present a vPCI device information.
 *
 * This structure includes the following information: the pointer to the vPCI field of a VM and the pointer to its
 * device configuration data; and the basic information of its self: like its BAR, MSI, virtual configuration space
 * registers and virtual BDF, and physical BDF if it is associated with a phyiscal PCI device.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct pci_vdev {
	struct acrn_vpci *vpci;
	/** The bus/device/function triple of the virtual PCI device. */
	union pci_bdf bdf;

	struct pci_pdev *pdev;

	union pci_cfgdata cfgdata;

	uint32_t flags;

	/** The bar info of the virtual PCI device. */
	uint32_t nr_bars; /** 6 for normal device, 2 for bridge, 1 for cardbus. */
	struct pci_vbar vbars[PCI_BAR_COUNT];

	uint8_t	prev_capoff; /** Offset of previous vPCI capability. */
	uint8_t	free_capoff; /** Next free offset to add vPCI capability. */

	struct pci_msi msi;
	struct pci_msix msix;
	struct pci_cap_sriov sriov;

	/** Pointer to the SRIOV VF associated PF's vdev. */
	struct pci_vdev *phyfun;

	/** Pointer to corresponding PCI device's vm_config. */
	struct acrn_vm_pci_dev_config *pci_dev_config;

	/** Pointer to corressponding operations. */
	const struct pci_vdev_ops *vdev_ops;

	/**
	 * vdev in    |   HV       |   pre-VM       |          Service VM                        | post-VM
	 *            |            |                |vdev used by Service VM|vdev used by post-VM|
	 * ----------------------------------------------------------------------------------------------------------
	 * parent_user| NULL(HV)   |   NULL(HV)     |   NULL(HV)            |   NULL(HV)         | vdev in Service VM
	 * ----------------------------------------------------------------------------------------------------------
	 * user       | vdev in HV | vdev in pre-VM |   vdev in Service VM  |   vdev in post-VM  | vdev in post-VM
	 */
	struct pci_vdev *parent_user;
	struct pci_vdev *user;	/** NULL means this device is not used or is a zombie VF. */
	struct hlist_node link;
	void *priv_data;
};

/**
 * @brief A union data structure to present a MMIO address used when a guest OS accesses a PCI configuration register.
 *
 * This union presents an address used when a guest OS accesses a PCI configuration register. It includes
 * the BDF of a PCI device, the register offset in the configuration space, and a enabling bit.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
union pci_cfg_addr_reg {
	uint32_t value;
	struct {
		uint32_t reg_num : 8;	/** BITs 0-7, Register Number (BITs 0-1, always reserve to 0). */
		uint32_t bdf : 16;	/** BITs 8-23, BDF Number. */
		uint32_t resv : 7;	/** BITs 24-30, Reserved. */
		uint32_t enable : 1;	/** BITs 31, Enable bit. */
	} bits;
};

/**
 * @brief Start address and end address of MMIO BAR.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct pci_mmio_res {
	uint64_t start;
	uint64_t end;
};

/**
 * @brief Data structure to present the vPCI information of a VM.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct acrn_vpci {
	spinlock_t lock;
	union pci_cfg_addr_reg addr;
	struct pci_mmcfg_region pci_mmcfg;
	uint32_t pci_vdev_cnt;
	struct pci_mmio_res res32; 	/** 32-bit mmio start/end address. */
	struct pci_mmio_res res64; 	/** 64-bit mmio start/end address. */
	struct pci_vdev pci_vdevs[CONFIG_MAX_PCI_DEV_NUM];
	struct hlist_head vdevs_hlist_heads [VDEV_LIST_HASHSIZE];
};

struct acrn_vm;

extern const struct pci_vdev_ops vhostbridge_ops;
extern const struct pci_vdev_ops vpci_bridge_ops;
extern const struct pci_vdev_ops vpci_mf_dev_ops;

/**
 * @brief Initialize the virtual PCI devices for the input VM
 *
 * This function is the top level virtual PCI initialization for the VM:
 * - Set up IOMMU domain;
 * - Get VM PCI configs and set up;
 * - Build up all virtual PCI devices given to this VM and register MMIO/PIO access handlers.
 *
 * @param[in,out] vm The target VM's acrn_vm structure.
 *
 * @return 0 If success, error codes if errors.
 *
 * @retval 0 Success.
 * @retval -EIO If one of the virtual PCI device's PIO BAR isn't indentical mapping between host and guest addresses.
 *
 * @pre vm != NULL
 * @pre vm->vm_id < CONFIG_MAX_VM_NUM
 *
 * @post vm's PCI related configs and internal PCI device list are set up.
 */
int32_t init_vpci(struct acrn_vm *vm);

/**
 * @brief Deinitialize the virtual PCI devices for the input VM
 *
 * This functions is the top level virtual PCI deinitialization for the VM:
 * - Deinit each virtual PCI device from the VM;
 * - Unmap IRQs of passthrough PCI devices;
 * - Clean up IOMMU setting for the VM.
 * This is the reverse of init_vpci.
 *
 * @param[in,out] vm The target VM's acrn_vm structure.
 *
 * @return N/A
 *
 * @pre vm != NULL
 * @pre vm->vm_id < CONFIG_MAX_VM_NUM
 *
 * @post vm's PCI related configs and internal PCI device list are cleaned up.
 */
void deinit_vpci(struct acrn_vm *vm);

/**
 * @brief Find a PCI device in the acrn_vpci internal device list by PCI BDF
 *
 * @param[in] vpci The data structure of acrn_vpci to be searched in.
 * @param[in] vbdf The PCI BDF to be searched.
 *
 * @return The pci_vdev structure pointer if found, NULL if not.
 *
 * @pre vpci != NULL
 * @pre vpci->pci_vdev_cnt <= CONFIG_MAX_PCI_DEV_NUM
 */
struct pci_vdev *pci_find_vdev(struct acrn_vpci *vpci, union pci_bdf vbdf);

struct acrn_pcidev;

/**
 * @brief Assign a PCI device from Service VM to target post-launched VM
 *
 * @param[in] tgt_vm Target post-launched VM.
 * @param[in] pcidev The PCI device to be assigned.
 *
 * @return 0 If success, error codes if errors.
 *
 * @retval 0        PCI device is assigned to target VM successfully.
 * @retval -EIO     PCI device's PIO BAR isn't identical mapping of host address and guest VM address.
 * @retval -ENODEV  PCI device is not found.
 *
 * @pre tgt_vm != NULL
 * @pre pcidev != NULL
 */
int32_t vpci_assign_pcidev(struct acrn_vm *tgt_vm, struct acrn_pcidev *pcidev);

/**
 * @brief Deassign a PCI device from target post-launched VM to Service VM
 *
 * @param[in] tgt_vm Target post-launched VM.
 * @param[in] pcidev The PCI device to be deassigned.
 *
 * @return 0 if success, error codes if errors.
 *
 * @retval 0        PCI device is deassigned from target VM successfully.
 * @retval -ENODEV  PCI device is not found.
 *
 * @pre tgt_vm != NULL
 * @pre pcidev != NULL
 */
int32_t vpci_deassign_pcidev(struct acrn_vm *tgt_vm, struct acrn_pcidev *pcidev);

/**
 * @brief Initialize a pci_vdev structure
 *
 * The function vpci_init_vdev is used to initialize a pci_vdev structure with a PCI device configuration(dev_config)
 * on a specified vPCI bus(vpci). If the function vpci_init_vdev initializes a SRIOV Virtual Function(VF) vdev
 * structure, the parameter parent_pf_vdev is the VF associated Physical Function(PF) pci_vdev structure, otherwise the
 * parameter parent_pf_vdev is NULL.
 *
 * @param[in,out] vpci       Pointer to a acrn_vpci structure
 * @param[in] dev_config     Pointer to a dev_config structure of the vdev
 * @param[in] parent_pf_vdev If the parameter def_config points to a SRIOV VF vdev, this parameter parent_pf_vdev
 *                           indicates the parent PF vdev. Otherwise, it is NULL.
 *
 * @return Pointer to the new initialized vdev structure.
 *
 * @pre vpci != NULL
 * @pre vpci.pci_vdev_cnt < CONFIG_MAX_PCI_DEV_NUM
 *
 * @post vpci internal pci_vdev list is updated with the new virtual device.
 *
 * @remark The caller of the function vpci_init_vdev should guarantee execution atomically.
 */
struct pci_vdev *vpci_init_vdev(struct acrn_vpci *vpci, struct acrn_vm_pci_dev_config *dev_config, struct pci_vdev *parent_pf_vdev);

/**
 * @brief Check whether input PCI BAR maps to IO space
 *
 * @param[in] vbar The input BAR structure.
 *
 * @return TRUE if the input BAR maps to IO space, otherwise FALSE.
 *
 * @pre vbar != NULL
 */
static inline bool is_pci_io_bar(struct pci_vbar *vbar)
{
        return ((vbar->bar_type.io_space.indicator == 1U) && (!vbar->is_mem64hi));
}

/**
 * @brief Check whether input PCI BAR maps to memory space
 *
 * @param[in] vbar The input BAR structure.
 *
 * @return TRUE if the input BAR maps to memory space, otherwise FALSE.
 *
 * @pre vbar != NULL
 */
static inline bool is_pci_mem_bar(struct pci_vbar *vbar)
{
        return ((vbar->is_mem64hi) || ((vbar->bar_type.mem_space.indicator == 0U)));
}

/**
 * @brief Check whether input PCI BAR is reserved
 *
 * Reserved PCI BAR type:
 * 1.Memory bar with reserved memory type;
 * 2.IO bar reserved bit is set.
 *
 * @param[in] vbar The input BAR structure.
 *
 * @return TRUE if the input BAR is reserved, otherwise FALSE.
 *
 * @pre vbar != NULL
 */
static inline bool is_pci_reserved_bar(struct pci_vbar *vbar)
{
        return (((vbar->bar_type.mem_space.indicator == 0U) && ((vbar->bar_type.mem_space.mem_type & 0x1U) == 0x1U) && (!vbar->is_mem64hi)) ||
		((vbar->bar_type.io_space.indicator == 1U) && (vbar->bar_type.io_space.reserved == 1U)));
}

/**
 * @brief Check whether input PCI BAR maps to memory space and its Base Address is 32 bits wide
 *
 * @param[in] vbar The input BAR structure.
 *
 * @return TRUE if the input BAR maps to 32 bits memory space, otherwise FALSE.
 *
 * @pre vbar != NULL
 */
static inline bool is_pci_mem32_bar(struct pci_vbar *vbar)
{
        return ((vbar->bar_type.mem_space.indicator == 0U) && (vbar->bar_type.mem_space.mem_type == 0U) && (!vbar->is_mem64hi));
}

/**
 * @brief Check whether input PCI BAR maps to 64 bits memory space and its Base Address maps to the low 32 bits address
 *
 * @param[in] vbar The input BAR structure.
 *
 * @return TRUE if the input BAR contains the low 32 bit address of 64 bits memory space, otherwise FALSE.
 *
 * @pre vbar != NULL
 */
static inline bool is_pci_mem64lo_bar(struct pci_vbar *vbar)
{
        return ((vbar->bar_type.mem_space.indicator == 0U) && (vbar->bar_type.mem_space.mem_type == 2U) && (!vbar->is_mem64hi));
}

/**
 * @}
 */

#endif /* VPCI_H_ */
