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

#ifndef VPCI_PRIV_H_
#define VPCI_PRIV_H_

#include <list.h>
#include <pci.h>

/**
 * @addtogroup vp-dm_vperipheral
 *
 * @{
 */

/**
 * @file
 * @brief This file declares functions, data structures and macros used by vPCI component internally
 *
 */

/*
 * @brief Pre-defined max entry number of vMSIX table
 *
 * For hypervisor emulated PCI devices, vMSIX Table contains 128 entries
 * at most. vMSIX Table begins at an offset of 0, and maps the vMSIX PBA
 * beginning at an offset of 2 KB.
 */
#define VMSIX_MAX_TABLE_ENTRY_NUM  128U
#define VMSIX_MAX_ENTRY_TABLE_SIZE 2048U /**< Pre-defined vMSIX table size */
#define VMSIX_ENTRY_TABLE_PBA_BAR_SIZE 4096U /**< Pre-defined vMSIX PBA table size */

static inline struct acrn_vm *vpci2vm(const struct acrn_vpci *vpci)
{
	return container_of(vpci, struct acrn_vm, vpci);
}

static inline bool is_quirk_ptdev(const struct pci_vdev *vdev)
{
	return ((vdev->flags & ACRN_PTDEV_QUIRK_ASSIGN) != 0U);
}

static inline bool in_range(uint32_t value, uint32_t lower, uint32_t len)
{
	return ((value >= lower) && (value < (lower + len)));
}

/**
 * @pre vdev != NULL
 */
static inline bool has_msix_cap(const struct pci_vdev *vdev)
{
	return (vdev->msix.capoff != 0U);
}

/**
 * @pre vdev != NULL
 */
static inline bool msixcap_access(const struct pci_vdev *vdev, uint32_t offset)
{
	return (has_msix_cap(vdev) && in_range(offset, vdev->msix.capoff, vdev->msix.caplen));
}

/**
 * @pre vdev != NULL
 */
static inline bool msixtable_access(const struct pci_vdev *vdev, uint32_t offset)
{
	return in_range(offset, vdev->msix.table_offset, vdev->msix.table_count * MSIX_TABLE_ENTRY_SIZE);
}

/*
 * @pre vdev != NULL
 */
static inline bool has_sriov_cap(const struct pci_vdev *vdev)
{
	return (vdev->sriov.capoff != 0U);
}

/*
 * @pre vdev != NULL
 */
static inline bool sriovcap_access(const struct pci_vdev *vdev, uint32_t offset)
{
	return (has_sriov_cap(vdev) && in_range(offset, vdev->sriov.capoff, vdev->sriov.caplen));
}

/**
 * @pre vdev != NULL
 */
static inline bool vbar_access(const struct pci_vdev *vdev, uint32_t offset)
{
	return is_bar_offset(vdev->nr_bars, offset);
}

/**
 * @pre vdev != NULL
 */
static inline bool cfg_header_access(uint32_t offset)
{
	return (offset < PCI_CFG_HEADER_LENGTH);
}

/**
 * @pre vdev != NULL
 */
static inline bool has_msi_cap(const struct pci_vdev *vdev)
{
	return (vdev->msi.capoff != 0U);
}

/**
 * @pre vdev != NULL
 */
static inline bool msicap_access(const struct pci_vdev *vdev, uint32_t offset)
{
	return (has_msi_cap(vdev) && in_range(offset, vdev->msi.capoff, vdev->msi.caplen));
}

/**
 * @brief Check if the specified vdev is a zombie VF instance
 *
 * @param[in] vdev Pointer to vdev instance.
 *
 * @return If the vdev is a zombie VF instance return true, otherwise return false.
 *
 * @pre vdev != NULL
 */
static inline bool is_zombie_vf(const struct pci_vdev *vdev)
{
	return (vdev->user == NULL);
}

void init_vdev_pt(struct pci_vdev *vdev, bool is_pf_vdev);
void deinit_vdev_pt(struct pci_vdev *vdev);
void vdev_pt_write_vbar(struct pci_vdev *vdev, uint32_t idx, uint32_t val);
void vdev_pt_map_msix(struct pci_vdev *vdev, bool hold_lock);

void init_vmsi(struct pci_vdev *vdev);
void write_vmsi_cap_reg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t val);
void deinit_vmsi(const struct pci_vdev *vdev);

void init_vmsix_pt(struct pci_vdev *vdev);
int32_t add_vmsix_capability(struct pci_vdev *vdev, uint32_t entry_num, uint8_t bar_num);
void read_vmsix_cap_reg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t *val);
bool write_vmsix_cap_reg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t val);
void read_pt_vmsix_cap_reg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t *val);
void write_pt_vmsix_cap_reg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t val);
uint32_t rw_vmsix_table(struct pci_vdev *vdev, struct io_request *io_req);
int32_t vmsix_handle_table_mmio_access(struct io_request *io_req, void *priv_data);
bool vpci_vmsix_enabled(const struct pci_vdev *vdev);
void deinit_vmsix_pt(struct pci_vdev *vdev);

void init_vmsix_on_msi(struct pci_vdev *vdev);
void write_vmsix_cap_reg_on_msi(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t val);
void remap_one_vmsix_entry_on_msi(struct pci_vdev *vdev, uint32_t index);

/**
 * @brief Initialize SRIOV device, i.e. set capability register offset and length
 *
 * @param[in] vdev Pointer to SRIOV device.
 *
 * @return None.
 *
 * @pre vdev != NULL
 * @pre vdev->pdev != NULL
 */
void init_vsriov(struct pci_vdev *vdev);

/**
 * @brief Read SRIOV device register value
 *
 * @param[in]  vdev   Pointer to SRIOV device.
 * @param[in]  offset Offset of the register to read.
 * @param[in]  bytes  Bytes to read.
 * @param[out] val    Buffer of value to be read out.
 *
 * @return None.
 *
 * @pre vdev != NULL
 * @pre vdev->pdev != NULL
 */
void read_sriov_cap_reg(const struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t *val);

/**
 * @brief Write value to SRIOV device register
 *
 * @param[in] vdev   Pointer to SRIOV device.
 * @param[in] offset Offset of the register to write.
 * @param[in] bytes  Bytes to write.
 * @param[in] val    Value to writet.
 *
 * @return None.
 *
 * @pre vdev != NULL
 * @pre vdev->pdev != NULL
 */
void write_sriov_cap_reg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t val);

/**
 * @brief Get offset of VF BAR
 *
 * @param[in] pf_vdev Pointer to physical function device.
 * @param[in] bar_idx VF BAR ID to get offset.
 *
 * @return VF BAR offset.
 *
 * @pre pf_vdev != NULL
 */
uint32_t sriov_bar_offset(const struct pci_vdev *vdev, uint32_t bar_idx);

/**
 * @brief Read virtual PCI device config
 *
 * @param[in] vdev   Pointer to vdev instance.
 * @param[in] offset The offset of the virtual PCI device config space to read.
 * @param[in] bytes  The size to read.
 *
 * @return The config value to read.
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
uint32_t pci_vdev_read_vcfg(const struct pci_vdev *vdev, uint32_t offset, uint32_t bytes);

/**
 * @brief Write value to virtual PCI device config
 *
 * @param[in,out] vdev   Pointer to vdev data structure.
 * @param[in]     offset The offset of the virtual PCI device config space to write.
 * @param[in]     bytes  The size to write.
 * @param[in]     val    The value to write.
 *
 * @return None.
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
void pci_vdev_write_vcfg(struct pci_vdev *vdev, uint32_t offset, uint32_t bytes, uint32_t val);

/**
 * @brief Add emulated legacy PCI capability support for virtual PCI device
 *
 * @param[in,out] vdev    Pointer to vdev data structure
 * @param[in]     capdata Pointer to buffer that holds the capability data to be added.
 * @param[in]     caplen  Length of buffer that holds the capability data to be added.
 *
 * @pre vdev != NULL
 * @pre vdev->vpci != NULL
 */
uint32_t vpci_add_capability(struct pci_vdev *vdev, uint8_t *capdata, uint8_t caplen);

/**
 * @brief Write value to a BAR of virtual PCI device
 *
 * @param[in,out] vdev Pointer to vdev instance.
 * @param[in]     idx  The idx of the BAR to update.
 * @param[in]     val  The value to write.
 *
 * @return None.
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
void pci_vdev_write_vbar(struct pci_vdev *vdev, uint32_t idx, uint32_t val);

void vdev_pt_hide_sriov_cap(struct pci_vdev *vdev);

/**
 * @brief Check if the passthr device BARs are valid PIO BARs
 *
 * @param[in] vdev Pointer to vdev instance.
 *
 * @return 0 for success, otherwise -EIO.
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
int32_t check_pt_dev_pio_bars(struct pci_vdev *vdev);

/**
 * @brief Function pointer typedef of PCI BAR map/unmap functions.
 *
 * These two function pointers define the vpci_update_one_vbar() arguments of PCI BAR map/unmap callbacks.
 */
typedef void (*map_pcibar)(struct pci_vdev *vdev, uint32_t bar_idx);
typedef void (*unmap_pcibar)(struct pci_vdev *vdev, uint32_t bar_idx);

/*
 * @brief Update one BAR of the input virual PCI device.
 *
 * The purpose of this function is to remap the target BAR address of the input PCI device with pre/post
 * processing around updating the Base Address. The \a unmap_cb callback should be provided and will be called before
 * updating the BAR Base Address. The \a map_cb callback will be called after updating the BAR Base Address if the
 * callback exists.
 *
 * @param[in,out] vdev     The target virtual PCI device.
 * @param[in]     bar_idx  The target BAR's index.
 * @param[in]     val      The value to be written to the target BAR's Base Address.
 * @param[in]     map_cb   The map callback for the target BAR.
 * @param[in]     unmap_cb The unmap callback for the target BAR.
 *
 * @return N/A
 *
 * @pre unmap_cb != NULL
 *
 * @post vdev->vbars[bar_idx] (or vdev->vbars[bar_idx - 1] if vdev->vbars[bar_idx]->is_mem64hi == TRUE) is updated.
 */
void vpci_update_one_vbar(struct pci_vdev *vdev, uint32_t bar_idx, uint32_t val, map_pcibar map_cb, unmap_pcibar unmap_cb);

/**
 * @}
 */

#endif /* VPCI_PRIV_H_ */
