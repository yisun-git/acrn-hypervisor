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

#ifndef IVSHMEM_H
#define IVSHMEM_H

/**
 * @addtogroup vp-dm_vperipheral
 *
 * @{
 */

/**
 * @file
 * @brief This file declares ivshmem device external APIs provided by the virtual PCI (vPCI) component.
 *
 * This file declares ivshmem device external functions, data structures, and macros that shall be provided by the vPCI
 * component in vp-dm.vperipheral module.
 *
 */

#define	IVSHMEM_VENDOR_ID	0x1af4U /**< Pre-defined "Vendor ID" of ivshmem device */
#define	IVSHMEM_DEVICE_ID	0x1110U /**< Pre-defined "Device ID" of ivshmem device */

#ifdef CONFIG_IVSHMEM_ENABLED

/**
 * @brief Pre-defined max number of peers for each ivshmem region
 *
 * Max number of peers for each ivshmem region, and
 * VM ID is used as peer IDs of this VM's ivshmem devices.
 */
#define MAX_IVSHMEM_PEER_NUM (CONFIG_MAX_VM_NUM)

#define MAX_IVSHMEM_MSIX_TBL_ENTRY_NUM 8U /**< Pre-defined max number of MSIX table entries of shmem device. */

/**
 * @brief Data structure to present ivshmem device shared memory region information
 *
 * For a ivshmem device shared memory region, this structure includes its name,
 * HPA of region, region size and the array to keep the ivshmem devices of the
 * region.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct ivshmem_shm_region {
	char name[32];
	uint64_t hpa;
	uint64_t size;
	struct ivshmem_device *doorbell_peers[MAX_IVSHMEM_PEER_NUM];
};

extern const struct pci_vdev_ops vpci_ivshmem_ops;

/**
 * @brief Initialize ivshmem shared memory regions
 *
 * Initialize ivshmem shared memory regions based on user configuration.
 *
 * @return None
 *
 * @pre N/A
 *
 * @post N/A
 *
 * @remark N/A
 */
void init_ivshmem_shared_memory(void);

/*
 * @brief Create an ivshmem device for a VM
 *
 * This function creates an ivshmem device as vPCI device for a VM and initialize it.
 *
 * @param[in,out] vm  Pointer to VM to create ivshmem device.
 * @param[in]     dev Pointer to virtual device that the VM wants to create.
 *
 * @return 0 if no error happens otherwise return -EINVAL.
 *
 * @pre vm != NULL
 * @pre dev != NULL
 *
 * @remark N/A
 */
int32_t create_ivshmem_vdev(struct acrn_vm *vm, struct acrn_vdev *dev);

/*
 * @brief Destroy an ivshmem device
 *
 * @param[in,out] vdev Pointer to the virtual PCI device to destroy.
 *
 * @return 0
 *
 * @pre vdev != NULL
 *
 * @remark N/A
 */
int32_t destroy_ivshmem_vdev(struct pci_vdev *vdev);

#endif /* CONFIG_IVSHMEM_ENABLED */

/**
 * @}
 */

#endif /* IVSHMEM_H */
