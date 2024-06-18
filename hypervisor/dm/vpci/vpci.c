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

#include <errno.h>
#include <ptdev.h>
#include <asm/guest/vm.h>
#include <asm/vtd.h>
#include <asm/io.h>
#include <asm/mmu.h>
#include <vacpi.h>
#include <logmsg.h>
#include "vpci_priv.h"
#include <asm/pci_dev.h>
#include <hash.h>
#include <board_info.h>

/**
 * @file vpci.c
 * @brief Implementation of the vPCI (Virtual PCI) module.
 *
 * This file contains the implementation of the vPCI module, which provides
 * functionality for virtualizing PCI devices in the ACRN Hypervisor.
 */

static int32_t vpci_init_vdevs(struct acrn_vm *vm);
static int32_t vpci_read_cfg(struct acrn_vpci *vpci, union pci_bdf bdf, uint32_t offset, uint32_t bytes, uint32_t *val);
static int32_t vpci_write_cfg(struct acrn_vpci *vpci, union pci_bdf bdf, uint32_t offset, uint32_t bytes, uint32_t val);
static struct pci_vdev *find_available_vdev(struct acrn_vpci *vpci, union pci_bdf bdf);

/**
 * @brief Reads the PCI configuration address register value for virtual device PIO (Port I/O) access.
 *
 * This function is responsible for reading the virtual PCI device's PCI configuration
 * address register value for PIO access.
 *
 * The function takes three parameters: a pointer to a structure called acrn_vcpu, an unsigned
 * 16-bit integer addr, and a size_t variable bytes.
 *
 * Inside the function, there are several local variables declared. The variable val is
 * initialized to the maximum value of an unsigned 32-bit integer. Another pointer cfg_addr
 * is assigned to the PCI configuration address register value got from vcpu->vm->vpci.addr.
 * Additionally, a pointer pio_req is declared to point to address of
 * vcpu->req.reqs.pio_request.
 *
 * The code then checks if the addr parameter is equal to the constant value PCI_CONFIG_ADDR
 * and if the bytes parameter is equal to 4. If both conditions are true, the value of the
 * value member of the cfg_addr structure is assigned to the val variable.
 *
 * Finally, the value member of the pio_req structure is assigned the value of val, and the
 * function returns true.
 *
 * @param[in,out] vcpu  The pointer to the virtual CPU structure.
 * @param[in]     addr  The address to read from.
 * @param[in]     bytes The number of bytes to read.
 *
 * @return always true.
 *
 * @pre vcpu != NULL
 * @pre vcpu->vm != NULL
 */
static bool vpci_pio_cfgaddr_read(struct acrn_vcpu *vcpu, uint16_t addr, size_t bytes)
{
	uint32_t val = ~0U;
	struct acrn_vpci *vpci = &vcpu->vm->vpci;
	union pci_cfg_addr_reg *cfg_addr = &vpci->addr;
	struct acrn_pio_request *pio_req = &vcpu->req.reqs.pio_request;

	if ((addr == (uint16_t)PCI_CONFIG_ADDR) && (bytes == 4U)) {
		val = cfg_addr->value;
	}

	pio_req->value = val;

	return true;
}

/**
 * @brief Handles the virtual device PIO (Port I/O) write operation to the PCI configuration address register.
 *
 * This function is responsible for handling the PIO write operation to the PCI configuration address register.
 *
 * The function takes four parameters: a pointer to a structure called acrn_vcpu, an
 * unsigned 16-bit integer addr, a size_t variable bytes, and an unsigned 32-bit integer
 * val.
 *
 * The function begins with the declaration of a boolean variable ret and a pointer to a
 * struct acrn_vpci object named vpci. The acrn_vpci structure represents the virtual PCI
 * controller.
 *
 * Next, there is a declaration of a union called pci_cfg_addr_reg which is used to access
 * the PCI configuration address register. This union allows accessing the register as a
 * whole or as individual bit fields. The cfg_addr variable is a pointer to this union.
 *
 * Another union called pci_bdf is declared to represent the bus, device, and function
 * numbers of a PCI device. This union is used to extract the BDF (Bus, Device, Function)
 * value from the cfg_addr union.
 *
 * The code checks if the provided addr parameter is equal to the predefined value
 * PCI_CONFIG_ADDR and if the bytes parameter is equal to 4. If both conditions are
 * true, it proceeds with further processing.
 *
 * Inside the condition, the code masks out the reserved fields of the val parameter
 * by performing a bitwise AND operation with the complement of the value 0x7f000003U.
 * This operation effectively clears bits 24-30 and bits 0-1 of the val parameter.
 *
 * If the virtual machine associated with the current vCPU is a post-launched VM, the
 * code proceeds with additional checks. It retrieves the BDF value from the cfg_addr
 * union and searches for an available virtual PCI device using the find_available_vdev
 * function. If the device is not found or if it is a "quirk" PT (Passthrough) device,
 * the ret variable is set to false.
 *
 * Finally, the function returns the value of the ret variable, indicating whether the
 * write operation was successful or not.
 *
 * @param[in,out] vcpu  The pointer to the acrn_vcpu structure representing the virtual CPU.
 * @param[in]     addr  The address value written to the PCI configuration address register.
 * @param[in]     bytes The number of bytes written to the PCI configuration address register.
 * @param[in]     val   The value written to the PCI configuration address register.
 *
 * @return true if the write operation is successful, false otherwise.
 *
 * @pre vcpu != NULL
 * @pre vcpu->vm != NULL
 */
static bool vpci_pio_cfgaddr_write(struct acrn_vcpu *vcpu, uint16_t addr, size_t bytes, uint32_t val)
{
	bool ret = true;
	struct acrn_vpci *vpci = &vcpu->vm->vpci;
	union pci_cfg_addr_reg *cfg_addr = &vpci->addr;
	union pci_bdf vbdf;

	if ((addr == (uint16_t)PCI_CONFIG_ADDR) && (bytes == 4U)) {
		/* unmask reserved fields: BITs 24-30 and BITs 0-1 */
		cfg_addr->value = val & (~0x7f000003U);

		if (is_postlaunched_vm(vcpu->vm)) {
			const struct pci_vdev *vdev;

			vbdf.value = cfg_addr->bits.bdf;
			vdev = find_available_vdev(vpci, vbdf);
			/* For post-launched VM, ACRN HV will only handle PT device,
			 * all virtual PCI device and QUIRK PT device
			 * still need to deliver to ACRN DM to handle.
			 */
			if ((vdev == NULL) || is_quirk_ptdev(vdev)) {
				ret = false;
			}
		}
	}

	return ret;
}

/**
 * @brief Reads PCI configuration data from virtual device PIO (Port I/O) space.
 *
 * This function reads PCI configuration data from the PIO space.
 *
 * It takes a virtual CPU and the address and size of the data to be read as parameters.
 *
 * Inside the function, there are several local variables declared, such as ret, vm, vpci,
 * cfg_addr, bdf, val, and pio_req. These variables are used to store intermediate values
 * and perform calculations within the function. The pointer vm is declared to point to
 * address of vcpu->vm. The pointer vpci is declared to point to address of vm->vpci. The
 * pointer pio_req is declared to point to address of vcpu->req.reqs.pio_request.
 *
 * The function begins by assigning the value of vpci->addr.value to cfg_addr.value using
 * an atomic read and clear operation. This means that the value of vpci->addr.value is
 * read and then cleared to 0 in an atomic manner, ensuring thread safety.
 *
 * Next, the function checks if the enable bit of cfg_addr is not equal to 0. If it is not
 * zero, it proceeds to calculate an offset value based on the reg_num field of cfg_addr
 * and the difference between addr and PCI_CONFIG_DATA. This offset is used to determine
 * the specific configuration register being accessed.
 *
 * The function then checks if the calculated offset and the number of bytes being accessed
 * are valid using the pci_is_valid_access function. If the access is valid, it retrieves
 * the bdf (bus, device, function) value from cfg_addr and calls the vpci_read_cfg function
 * to read the configuration data from the specified bdf, offset, and number of bytes. The
 * result of this read operation is stored in the val variable.
 *
 * Finally, the value of val is assigned to pio_req->value, which is a member of the
 * acrn_pio_request structure. The function returns true if ret is equal to 0, indicating a
 * successful read operation.
 *
 * @param[in,out] vcpu  The virtual CPU from which to read the data.
 * @param[in]     addr  The address of the data to be read.
 * @param[in]     bytes The size of the data to be read.
 *
 * @return True if the read operation is successful, false otherwise.
 *
 * @pre vcpu != NULL
 * @pre vcpu->vm != NULL
 * @pre vcpu->vm->vm_id < CONFIG_MAX_VM_NUM
 */
static bool vpci_pio_cfgdata_read(struct acrn_vcpu *vcpu, uint16_t addr, size_t bytes)
{
	int32_t ret = 0;
	struct acrn_vm *vm = vcpu->vm;
	struct acrn_vpci *vpci = &vm->vpci;
	union pci_cfg_addr_reg cfg_addr;
	union pci_bdf bdf;
	uint32_t val = ~0U;
	struct acrn_pio_request *pio_req = &vcpu->req.reqs.pio_request;

	cfg_addr.value = atomic_readandclear32(&vpci->addr.value);
	if (cfg_addr.bits.enable != 0U) {
		uint32_t offset = (uint16_t)cfg_addr.bits.reg_num + (addr - PCI_CONFIG_DATA);
		if (pci_is_valid_access(offset, bytes)) {
			bdf.value = cfg_addr.bits.bdf;
			ret = vpci_read_cfg(vpci, bdf, offset, bytes, &val);
		}
	}

	pio_req->value = val;
	return (ret == 0);
}

/**
 * @brief Writes data to the virtual device PCI configuration space using PIO (Port I/O) method.
 *
 * This function is responsible for writing data to the PCI configuration space
 * of a specific virtual PCI device using the PIO method.
 *
 * The function takes in several parameters: a pointer to a structure called acrn_vcpu,
 * an unsigned 16-bit integer addr, a size_t variable bytes, and an unsigned 32-bit
 * integer val.
 *
 * Inside the function, there are some variable declarations. ret is an integer
 * variable initialized to 0, vm is a pointer to a structure called acrn_vm, and
 * vpci is a pointer to a structure called acrn_vpci. The pointer vm is declared to
 * point to address of vcpu->vm. The pointer vpci is declared to point to address
 * of vm->vpci.
 *
 * Next, it retrieves the value of the addr member from the vpci structure and assigns
 * it to the cfg_addr union variable. The atomic_readandclear32 function is used to
 * read and clear the value atomically, ensuring thread safety.
 *
 * The code then checks if the enable member of the cfg_addr structure is not equal
 * to 0. If it is not zero, it means that the PCI configuration address is enabled.
 * In this case, the code proceeds to calculate the offset by adding the reg_num
 * member of cfg_addr with the difference between addr and PCI_CONFIG_DATA. This
 * offset represents the location within the PCI configuration space where the write
 * operation should be performed.
 *
 * The pci_is_valid_access function is called to check if the calculated offset and
 * the number of bytes to write are valid for the PCI configuration space. If the
 * access is valid, the code retrieves the bdf member from the cfg_addr structure
 * and calls the vpci_write_cfg function to perform the actual write operation on
 * the virtual PCI device.
 *
 * Finally, the function returns a boolean value indicating whether the write operation
 * was successful. It checks if the value of ret is equal to 0, which implies that the
 * write operation completed without any errors.
 *
 * @param[in,out] vcpu  The pointer to the virtual CPU structure.
 * @param[in]     addr  The address within the PCI configuration space to write to.
 * @param[in]     bytes The number of bytes to write.
 * @param[in]     val   The value to write.
 *
 * @return true if the write operation is successful, false otherwise.
 *
 * @pre vcpu != NULL
 * @pre vcpu->vm != NULL
 * @pre vcpu->vm->vm_id < CONFIG_MAX_VM_NUM
 */
static bool vpci_pio_cfgdata_write(struct acrn_vcpu *vcpu, uint16_t addr, size_t bytes, uint32_t val)
{
	int32_t ret = 0;
	struct acrn_vm *vm = vcpu->vm;
	struct acrn_vpci *vpci = &vm->vpci;
	union pci_cfg_addr_reg cfg_addr;
	union pci_bdf bdf;

	cfg_addr.value = atomic_readandclear32(&vpci->addr.value);
	if (cfg_addr.bits.enable != 0U) {
		uint32_t offset = (uint16_t)cfg_addr.bits.reg_num + (addr - PCI_CONFIG_DATA);
		if (pci_is_valid_access(offset, bytes)) {
			bdf.value = cfg_addr.bits.bdf;
			ret = vpci_write_cfg(vpci, bdf, offset, bytes, val);
		}
	}

	return (ret == 0);
}

/**
 * @brief Handle MMIO configuration access for the virtual PCI device
 *
 * The function takes two parameters: io_req, which is a pointer to a structure
 * representing an I/O request (io_req), and private_data, which is a pointer to some
 * private data specific to the vPCI instance.
 *
 * Inside the function, there are several local variables declared. ret is an
 * integer variable used to store the return value of the function. mmio is a
 * pointer to a structure representing a memory-mapped I/O (MMIO) request, it points
 * to the address of io_req->reqs.mmio_request. vpci is a pointer to a structure
 * representing the vPCI instance, the input private_data is assigned to it.
 * pci_mmcofg_base is a 64-bit unsigned integer representing the base address of the
 * PCI configuration space, i.e. vpci->pci_mmcfg.address. address is a 64-bit
 * unsigned integer representing the address of the MMIO request. reg_num is a 32-bit
 * unsigned integer representing the register number within the PCI configuration
 * space. Finally, bdf is a union of a 16-bit unsigned integer and a structure
 * representing the bus, device, and function numbers of a PCI device.
 *
 * The code then proceeds to calculate the bus, device, and function numbers based
 * on the provided MMIO address. This calculation involves subtracting the base
 * address of the PCI configuration space from the MMIO address and shifting the
 * result by 12 bits. The resulting values are stored in the bdf union.
 *
 * Next, the code checks the direction of the MMIO request. If it is a read request
 * (ACRN_IOREQ_DIR_READ), the code proceeds to read the value from the PCI
 * configuration space using the vpci_read_cfg function. This function takes the VPCI
 * instance, the bus/device/function numbers, the register number, the size of the
 * access, and a pointer to store the read value. If the access is valid (determined
 * by the pci_is_valid_access function), the vpci_read_cfg function is called, and the
 * read value is stored in the val variable. Finally, the read value is assigned to the
 * mmio->value field.
 *
 * On the other hand, if the MMIO request is a write request, the code checks for a
 * valid access and then calls the vpci_write_cfg function to write the provided value
 * to the PCI configuration space. The parameters passed to vpci_write_cfg are similar
 * to vpci_read_cfg, including the VPCI instance, bus/device/function numbers, register
 * number, size of the access, and the value to be written.
 *
 * Finally, the function returns the value of ret, which indicates the success or failure
 * of the MMIO configuration access.
 *
 * @param[in,out] io_req       Pointer to io_request.
 * @param[in]     private_data Pointer to acrn_vpci.
 *
 * @return 0 for success. Otherwise, false.
 *
 * @pre io_req != NULL && private_data != NULL
 */
static int32_t vpci_mmio_cfg_access(struct io_request *io_req, void *private_data)
{
	int32_t ret = 0;
	struct acrn_mmio_request *mmio = &io_req->reqs.mmio_request;
	struct acrn_vpci *vpci = (struct acrn_vpci *)private_data;
	uint64_t pci_mmcofg_base = vpci->pci_mmcfg.address;
	uint64_t address = mmio->address;
	uint32_t reg_num = (uint32_t)(address & 0xfffUL);
	union pci_bdf bdf;

	/**
	 * Enhanced Configuration Address Mapping
	 * A[(20+n-1):20] Bus Number 1 ≤ n ≤ 8
	 * A[19:15] Device Number
	 * A[14:12] Function Number
	 * A[11:8] Extended Register Number
	 * A[7:2] Register Number
	 * A[1:0] Along with size of the access, used to generate Byte Enables
	 */
	bdf.value = (uint16_t)((address - pci_mmcofg_base) >> 12U);

	if (mmio->direction == ACRN_IOREQ_DIR_READ) {
		uint32_t val = ~0U;

		if (pci_is_valid_access(reg_num, (uint32_t)mmio->size)) {
			ret = vpci_read_cfg(vpci, bdf, reg_num, (uint32_t)mmio->size, &val);
		}
		mmio->value = val;
	} else {
		if (pci_is_valid_access(reg_num, (uint32_t)mmio->size)) {
			ret = vpci_write_cfg(vpci, bdf, reg_num, (uint32_t)mmio->size, (uint32_t)mmio->value);
		}
	}

	return ret;
}

/**
 * @brief Initializes the virtual PCI (vPCI) for a given VM.
 *
 * This function is responsible for initializing the virtual PCI (Peripheral Component
 * Interconnect) configuration for a given virtual machine (struct acrn_vm) which is
 * input parameter.
 *
 * The function begins by defining two struct vm_io_range variables, pci_cfgaddr_range
 * and pci_cfgdata_range, which represent the ranges of I/O addresses for the PCI
 * configuration address and data. These ranges are used later in the function.
 *
 * First, it creates IOMMU domain for the input VM by calling create_iommu_domain. Only
 * 4 level page table is supported so the address width is set to 48. The translation
 * table address is the host physical address of vm->arch_vm.nworld_eptp.
 *
 * Next, the function retrieves the virtual machine configuration (vm_config) for the
 * given virtual machine. It then checks if the load order of the virtual machine is
 * a service VM. If it is, the function retrieves the physical PCI MMCONFIG
 * (Memory-Mapped Configuration Space) region (pci_mmcfg) and assigns it to the virtual
 * machine's vpci structure. Additionally, it sets the start and end addresses for 32-bit
 * and 64-bit memory regions (vm->vpci.res32 and vm->vpci.res64) based on predefined
 * constants.
 *
 * If the load order is not a service VM, the function assigns virtual addresses and bus
 * ranges to the vpci structure based on other predefined constants specific to user VMs.
 * It sets the start and end addresses for 32-bit and 64-bit memory regions (vm->vpci.res32
 * and vm->vpci.res64) based on predefined constants too.
 *
 * After setting up the virtual PCI configuration, the function calls vpci_init_vdevs to
 * initialize the virtual devices associated with the virtual machine. The return value
 * of this function is stored in the ret variable.
 *
 * If the initialization of virtual devices is successful (ret == 0), the function proceeds
 * to register various emulation handlers for MMIO (Memory-Mapped I/O) and PIO (Port I/O)
 * operations. These handlers allow the virtual machine to intercept and handle specific I/O
 * operations related to PCI configuration.
 *
 * The register_mmio_emulation_handler function is called to register an MMIO emulation handler
 * for the PCI MMCONFIG address range. This handler is responsible for handling read and write
 * operations to the PCI configuration space.
 *
 * Next, two PIO emulation handlers are registered using the register_pio_emulation_handler
 * function. These handlers are responsible for intercepting and handling read and write
 * operations to the PCI configuration address and data ports.
 *
 * Finally, a spinlock is initialized for the virtual PCI (vm->vpci.lock) to ensure thread
 * safety during concurrent access.
 *
 * The function returns the value of ret, which indicates the success or failure of the virtual
 * device initialization process.
 *
 * @param[in,out] vm The pointer to the acrn_vm structure representing the VM.
 *
 * @return 0 on success, or an error code on failure.
 *
 * @retval 0 Success.
 * @retval -EIO If one of the virtual PCI device's PIO BAR isn't indentical mapping between host and guest addresses.
 *
 * @pre vm != NULL
 * @pre vm->vm_id < CONFIG_MAX_VM_NUM
 */
int32_t init_vpci(struct acrn_vm *vm)
{
	struct vm_io_range pci_cfgaddr_range = {
		.base = PCI_CONFIG_ADDR,
		.len = 1U
	};

	struct vm_io_range pci_cfgdata_range = {
		.base = PCI_CONFIG_DATA,
		.len = 4U
	};

	struct acrn_vm_config *vm_config;
	struct pci_mmcfg_region *pci_mmcfg;
	int32_t ret = 0;

	vm->iommu = create_iommu_domain(vm->vm_id, hva2hpa(vm->arch_vm.nworld_eptp), 48U);

	vm_config = get_vm_config(vm->vm_id);
	/* virtual PCI MMCONFIG for Service VM is same with the physical value */
	if (vm_config->load_order == SERVICE_VM) {
		pci_mmcfg = get_mmcfg_region();
		vm->vpci.pci_mmcfg = *pci_mmcfg;
		vm->vpci.res32.start = MMIO32_START;
		vm->vpci.res32.end = MMIO32_END;
		vm->vpci.res64.start = MMIO64_START;
		vm->vpci.res64.end = MMIO64_END;
	} else {
		vm->vpci.pci_mmcfg.address = USER_VM_VIRT_PCI_MMCFG_BASE;
		vm->vpci.pci_mmcfg.start_bus = USER_VM_VIRT_PCI_MMCFG_START_BUS;
		vm->vpci.pci_mmcfg.end_bus = USER_VM_VIRT_PCI_MMCFG_END_BUS;
		vm->vpci.res32.start = USER_VM_VIRT_PCI_MEMBASE32;
		vm->vpci.res32.end = USER_VM_VIRT_PCI_MEMLIMIT32;
		vm->vpci.res64.start = USER_VM_VIRT_PCI_MEMBASE64;
		vm->vpci.res64.end = USER_VM_VIRT_PCI_MEMLIMIT64;
	}

	/* Build up vdev list for vm */
	ret = vpci_init_vdevs(vm);

	if (ret == 0) {
		register_mmio_emulation_handler(vm, vpci_mmio_cfg_access, vm->vpci.pci_mmcfg.address,
			vm->vpci.pci_mmcfg.address + get_pci_mmcfg_size(&vm->vpci.pci_mmcfg), &vm->vpci, false);

		/* Intercept and handle I/O ports CF8h */
		register_pio_emulation_handler(vm, PCI_CFGADDR_PIO_IDX, &pci_cfgaddr_range,
			vpci_pio_cfgaddr_read, vpci_pio_cfgaddr_write);

		/* Intercept and handle I/O ports CFCh -- CFFh */
		register_pio_emulation_handler(vm, PCI_CFGDATA_PIO_IDX, &pci_cfgdata_range,
			vpci_pio_cfgdata_read, vpci_pio_cfgdata_write);

		spinlock_init(&vm->vpci.lock);
	}

	return ret;
}

/**
 * @brief Deinitialize the virtual PCI (vPCI) subsystem for the input VM
 *
 * This function is responsible for deinitializing the virtual PCI (vPCI) subsystem for
 * a given virtual machine (struct acrn_vm) which is input parameter.
 *
 * The function starts with a loop that iterates over a maximum number of PCI devices
 * (CONFIG_MAX_PCI_DEV_NUM) associated with the virtual machine. Inside the loop, each PCI
 * device is accessed through a pointer vdev of type struct pci_vdev.
 *
 * The condition if (vdev->user == vdev) checks if the current PCI device is owned by the
 * virtual machine itself. If it is, the deinitialization process proceeds. Otherwise, it
 * skips to the next iteration.
 *
 * Within the deinitialization process, the deinit_vdev function is called on the current
 * PCI device vdev. This function is responsible for performing any necessary cleanup or
 * deinitialization specific to the PCI device.
 *
 * Next, the code checks if the current PCI device has a parent device (parent_vdev). If it
 * does, it acquires a spinlock associated with the parent device's VPCI subsystem, calls
 * the init_vdev function on the parent device, and releases the spinlock. This step ensures
 * that the parent device is properly reinitialized after deinitializing the child device.
 *
 * After the loop completes, the ptdev_release_all_entries function is called to release all
 * entries associated with PCI passthrough devices for the virtual machine. This function
 * releases irq remapping for PCI passthrough devices.
 *
 * Then zero out the memory occupied by the vpci structure within the virtual machine (vm).
 * This step ensures that any remaining data in the vpci structure is cleared.
 *
 * Finally, the destroy_iommu_domain function is called to destroy the IOMMU (Input-Output
 * Memory Management Unit) domain associated with the virtual machine's IOMMU. This function
 * zero out the iommu domain.
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
void deinit_vpci(struct acrn_vm *vm)
{
	struct pci_vdev *vdev, *parent_vdev;
	uint32_t i;

	for (i = 0U; i < CONFIG_MAX_PCI_DEV_NUM; i++) {
		vdev = (struct pci_vdev *) &(vm->vpci.pci_vdevs[i]);

		/* Only deinit the VM's own devices */
		if (vdev->user == vdev) {
			parent_vdev = vdev->parent_user;

			vdev->vdev_ops->deinit_vdev(vdev);

			if (parent_vdev != NULL) {
				spinlock_obtain(&parent_vdev->vpci->lock);
				parent_vdev->vdev_ops->init_vdev(parent_vdev);
				spinlock_release(&parent_vdev->vpci->lock);
			}
		}
	}

	ptdev_release_all_entries(vm);
	(void)memset(&vm->vpci, 0U, sizeof(struct acrn_vpci));

	/* Free iommu */
	destroy_iommu_domain(vm->iommu);
}

/**
 * @brief Assigns a virtual device to an IOMMU domain.
 *
 * This function is responsible for assigning a virtual device to an IOMMU domain
 * in the ACRN Hypervisor. It attaches the physical device associated with the virtual
 * device to the specified IOMMU domain.
 *
 * The function takes a pointer to a pci_vdev structure as its parameter.
 *
 * Within the function, there is a local variable ret of type int32_t and a pointer
 * vm of type acrn_vm. The vpci2vm function is used to retrieve the acrn_vm structure
 * associated with the vdev parameter.
 *
 * The move_pt_device function is then called with several arguments: NULL, vm->iommu,
 * (uint8_t)vdev->pdev->bdf.bits.b, and (uint8_t)(vdev->pdev->bdf.value & 0xFFU). The
 * move_pt_device is responsible for moving a physical device to a specific IOMMU domain.
 *
 * After calling move_pt_device, the return value ret is checked. If it is not equal to
 * zero, indicating an error, the panic function is called. This suggests that if the
 * assignment of the IOMMU domain fails, the program will terminate abruptly.
 *
 * @param[in] vdev The virtual device to be assigned to the IOMMU domain.
 *
 * @return N/A
 *
 * @pre vdev != NULL
 * @pre vdev->vpci != NULL
 * @pre vpci2vm(vdev->vpci)->iommu != NULL
 */
static void assign_vdev_pt_iommu_domain(struct pci_vdev *vdev)
{
	int32_t ret;
	struct acrn_vm *vm = vpci2vm(vdev->vpci);

	ret = move_pt_device(NULL, vm->iommu, (uint8_t)vdev->pdev->bdf.bits.b,
		(uint8_t)(vdev->pdev->bdf.value & 0xFFU));
	if (ret != 0) {
		panic("failed to assign iommu device!");
	}
}

/**
 * @brief Removes the virtual device from the IOMMU domain.
 *
 * This function removes the specified virtual device from the IOMMU domain of the
 * associated virtual machine. It calls the `move_pt_device` function to move the
 * physical device associated with the virtual device to a different IOMMU domain.
 * If the operation fails, a panic is triggered.
 *
 * The function takes a pointer to a pci_vdev structure as its parameter.
 *
 * Inside the function, there is a local variable ret of type int32_t that will be
 * used to store the return value of a function call. Another local variable vm is
 * declared and assigned the value of vpci2vm(vdev->vpci). This suggests that vpci2vm
 * is a function that converts a vpci object to a vm object, representing a virtual
 * machine.
 *
 * Then, it calls a function named move_pt_device with several arguments. This function
 * is responsible for moving a physical device from one IOMMU domain to another. The
 * arguments passed to move_pt_device include the IOMMU object (vm->iommu) that the device
 * should be removed from, NULL, and the bus and device numbers of the virtual PCI device
 * (vdev->pdev->bdf.bits.b and vdev->pdev->bdf.value & 0xFFU, respectively).
 *
 * After calling move_pt_device, the return value is checked against 0 to see if the
 * operation was successful. If the return value is not 0, indicating a failure, panic
 * is triggered.
 *
 * @param[in] vdev The virtual device to be removed from the IOMMU domain.
 *
 * @return N/A
 *
 * @pre vdev != NULL
 * @pre vdev->vpci != NULL
 * @pre vpci2vm(vdev->vpci)->iommu != NULL
 */
static void remove_vdev_pt_iommu_domain(const struct pci_vdev *vdev)
{
	int32_t ret;
	const struct acrn_vm *vm = vpci2vm(vdev->vpci);

	ret = move_pt_device(vm->iommu, NULL, (uint8_t)vdev->pdev->bdf.bits.b,
		(uint8_t)(vdev->pdev->bdf.value & 0xFFU));
	if (ret != 0) {
		/*
		 *TODO
		 * panic needs to be removed here
		 * Currently unassign_pt_device can fail for multiple reasons
		 * Once all the reasons and methods to avoid them can be made sure
		 * panic here is not necessary.
		 */
		panic("failed to unassign iommu device!");
	}
}

/**
 * @brief Finds an available virtual PCI device based on the given PCI bus, device, and function (BDF).
 *
 * The purpose of this function is to find an available virtual device (vdev) based
 * on a given PCI bus, device, and function (BDF).
 *
 * The function takes two parameters: a pointer to a structure acrn_vpci representing
 * the virtual PCI subsystem, and a union pci_bdf representing the BDF of the device
 * to be found.
 *
 * The function starts by calling another function called pci_find_vdev to find a
 * virtual device with the specified BDF identifier. If a virtual device is found
 * (vdev != NULL), the function proceeds to check if the device is being used by
 * another user (vdev->user != vdev).
 *
 * If the device is being used by another user, the function performs an additional
 * check. It compares the severity level of the current virtual machine (VM)
 * associated with the acrn_vpci structure (vpci2vm(vpci)->vm_id) with the severity
 * level of the user VM associated with the vdev structure (vpci2vm(vdev->user->vpci)->vm_id).
 * The severity level is a measure of the priority of a VM.
 *
 * If the severity level of the current VM is lower than the severity level of the
 * user VM, it means that the user VM has higher priority, and therefore, the current
 * VM should not have access to the device. In this case, the vdev pointer is set to
 * NULL to indicate that the device is not available.
 *
 * If the device is not being used any user, the vdev pointer is also set to NULL to
 * indicate that the device is not available.
 *
 * Finally, the function returns the vdev pointer, which either points to an available
 * virtual device or is NULL if no device is available.
 *
 * @param[in] vpci The VPCI instance to search in.
 * @param[in] bdf  The PCI bus, device, and function (BDF) of the device to find.
 *
 * @return A pointer to the found virtual PCI device if it is available, or NULL if it is not available.
 *
 * @pre vpci != NULL
 */
static struct pci_vdev *find_available_vdev(struct acrn_vpci *vpci, union pci_bdf bdf)
{
	struct pci_vdev *vdev = pci_find_vdev(vpci, bdf);

	if ((vdev != NULL) && (vdev->user != vdev)) {
		if (vdev->user != NULL) {
			/* the Service VM is able to access, if and only if the Service VM has higher severity than the User VM. */
			if (get_vm_severity(vpci2vm(vpci)->vm_id) <
					get_vm_severity(vpci2vm(vdev->user->vpci)->vm_id)) {
				vdev = NULL;
			}
		} else {
			vdev = NULL;
		}
	}

	return vdev;
}

/**
 * @brief Initializes a virtual PCI device for passthrough.
 *
 * The function initializes a virtual PCI device (struct pci_vdev) which is input
 * parameter.
 *
 * First, the function sets the parent_user and user fields of the vdev structure.
 * These fields are pointers that are used to keep track of the parent and user of
 * the virtual device. In this case, parent is set to NULL initially, indicating
 * that there is no parent associated with the device. The user is set to virtual
 * device itself.
 *
 * The code then proceeds to call several initialization functions in a specific order.
 *
 * init_vmsi(vdev) initializes the virtual MSI (Message Signaled Interrupt) support for
 * the device. MSI is a mechanism used to deliver interrupts to the device driver in a
 * more efficient manner.
 *
 * init_vmsix_pt(vdev) initializes the virtual MSI-X (Message Signaled Interrupts eXtended)
 * support for the device. MSI-X is an extension of MSI that allows for more interrupt
 * vectors and greater flexibility in interrupt handling.
 *
 * init_vsriov(vdev) initializes the virtual SR-IOV (Single Root I/O Virtualization) support
 * for the device. SR-IOV allows a single physical device to appear as multiple virtual
 * devices, each with its own resources and configurations.
 *
 * init_vdev_pt(vdev, false) initializes the virtual device in passthrough mode. This mode
 * allows the virtual device to directly access the underlying physical device without any
 * virtualization or emulation.
 *
 * Finally, the function calls assign_vdev_pt_iommu_domain(vdev) to assign an IOMMU
 * (Input-Output Memory Management Unit) domain to the virtual device. An IOMMU is responsible
 * for mapping virtual addresses to physical addresses and provides memory protection and
 * isolation for devices.
 *
 * @param[in,out] vdev The PCI virtual device to initialize.
 *
 * @return N/A
 *
 * @pre vdev != NULL
 */
static void vpci_init_pt_dev(struct pci_vdev *vdev)
{
	vdev->parent_user = NULL;
	vdev->user = vdev;

	/*
	 * Here init_vdev_pt() needs to be called after init_vmsix_pt() for the following reason:
	 * init_vdev_pt() will indirectly call has_msix_cap(), which
	 * requires init_vmsix_pt() to be called first.
	 */
	init_vmsi(vdev);
	init_vmsix_pt(vdev);
	init_vsriov(vdev);
	init_vdev_pt(vdev, false);

	assign_vdev_pt_iommu_domain(vdev);
}

/**
 * @brief Deinitializes a virtual PCI device for passthrough.
 *
 * This function deinitializes the input virtual PCI device (struct pci_vdev) by
 * performing the following steps:
 * 1. Calls the deinit_vdev_pt() function to deinitialize the virtual device's
 *    passthrough configuration.
 * 2. Calls the remove_vdev_pt_iommu_domain() function to remove the virtual
 *    device from the IOMMU domain.
 * 3. Calls the deinit_vmsix_pt() function to deinitialize the virtual device's
 *    MSI-X configuration.
 * 4. Calls the deinit_vmsi() function to deinitialize the virtual device's MSI
 *    configuration.
 * 5. Sets the 'user' and 'parent_user' pointers of the virtual device to NULL.
 *
 * @param[in,out] vdev Pointer to the PCI virtual device to deinitialize.
 *
 * @return N/A
 *
 * @pre vdev != NULL
 */
static void vpci_deinit_pt_dev(struct pci_vdev *vdev)
{
	deinit_vdev_pt(vdev);
	remove_vdev_pt_iommu_domain(vdev);
	deinit_vmsix_pt(vdev);
	deinit_vmsi(vdev);

	vdev->user = NULL;
	vdev->parent_user = NULL;
}

/**
 * @brief Data structure to store information about PCI config space header permissions.
 *
 * Each member provides specific details about the permissions for each 4-byte register
 * in the PCI config space header.
 *
 * It is used when the hypervisor needs to determine whether a register is pass-through
 * or virtualized, and whether it's read-only or writable.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct cfg_header_perm {
    /**
     * @brief Indicates the pass-through status for each 4-byte register.
     *
     * If bit x is set, it indicates that the corresponding 4 Bytes register for bit x is
	 * pass through to guest. Otherwise, it's virtualized.
     */
    uint32_t pt_mask;
    /**
     * @brief Indicates the read-only status for each 4-byte register.
     *
     * If bit x is set, it indicates that the corresponding 4 Bytes register for bit x is
	 * read-only. Otherwise, it's writable.
     */
    uint32_t ro_mask;
};

/**
 * @brief Instance of the cfg_header_perm structure to store information about PCI config space header permissions.
 *
 * This instance is used to specify the pass-through and read-only permissions for the PCI
 * config space header registers.
 */
static const struct cfg_header_perm cfg_hdr_perm = {
    /**
     * @brief Pass-through mask for PCI config space header registers.
     *
     * Only Command (0x04-0x05) and Status (0x06-0x07) Registers are pass-through.
     */
    .pt_mask = 0x0002U,
    /**
     * @brief Read-only mask for PCI config space header registers.
     *
     * Command (0x04-0x05) and Status (0x06-0x07) Registers and
     * Base Address Registers (0x10-0x27) are writable.
     */
    .ro_mask = (uint16_t)~0x03f2U
};

/**
 * @brief Reads the configuration header of a PCI device.
 *
 * This function reads the configuration header of a PCI device specified by
 * the given virtual device (vdev), offset, number of bytes, and stores the
 * result in the val parameter.
 *
 * The function takes four parameters: vdev, offset, bytes, and val. The vdev
 * parameter is a pointer to a structure representing the virtual PCI device.
 * The offset parameter specifies the offset within the configuration header
 * that needs to be read. The bytes parameter indicates the number of bytes to
 * read, and the val parameter is a pointer to store the read value.
 *
 * The function begins by initializing the ret variable to 0. This variable will
 * be used to store the return value of the function.
 *
 * Next, the code checks if the offset is equal to PCIR_BIOS and if the PCI device
 * has a specific quirk called quirk_ptdev. If both conditions are true, the function
 * sets ret to -ENODEV. This is a specific behavior for the PCIR_BIOS offset when
 * the quirk_ptdev is present.
 *
 * If the above condition is not met, the code checks if the access to the offset is
 * related to a virtual base address register (BAR) by calling the vbar_access function.
 * If the condition is true, the code further checks if the bytes parameter is 4 and if
 * the offset is 4-byte aligned. If both conditions are met, the function reads the
 * value from the PCI device's virtual configuration space using the pci_vdev_read_vcfg
 * function and stores it in the val variable. Otherwise, it sets val to the maximum
 * value of an unsigned integer (~0U).
 *
 * If the access is not related to a virtual BAR, the code checks if the offset is allowed
 * based on a permission bitmap called cfg_hdr_perm.pt_mask. If the offset is allowed,
 * the function reads the value from the physical PCI device's configuration space using
 * the pci_pdev_read_cfg function and stores it in the val variable. Additionally, if
 * the device is an assigned virtual function (VF) and the offset is PCIR_COMMAND, the
 * function sets the Memory Space Enable (MSE) bit in the value. This ensures that the
 * MSE bit is always set for an assigned VF.
 *
 * If the offset is not allowed based on the permission bitmap, the function reads the
 * value from the PCI device's virtual configuration space using the pci_vdev_read_vcfg
 * function and stores it in the val variable.
 *
 * Finally, the function returns the value of ret, which indicates the success or failure
 * of the function.
 *
 * @param[in]  vdev   The virtual device representing the PCI device.
 * @param[in]  offset The offset within the configuration header to read from.
 * @param[in]  bytes  The number of bytes to read.
 * @param[out] val    Pointer to store the read value.
 *
 * @return 0 on success, or -ENODEV if it tries to read PCIR_BIOS from QUIRK PT device.
 *
 * @pre vdev != NULL
 * @pre vdev->pdev != NULL
 * @pre val != NULL
 * @pre offset + bytes < PCI_CFG_HEADER_LENGTH
 */
static int32_t read_cfg_header(const struct pci_vdev *vdev,
		uint32_t offset, uint32_t bytes, uint32_t *val)
{
	int32_t ret = 0;

	if ((offset == PCIR_BIOS) && is_quirk_ptdev(vdev)) {
		/* the access of PCIR_BIOS is emulated for quirk_ptdev */
		ret = -ENODEV;
	} else if (vbar_access(vdev, offset)) {
		/* bar access must be 4 bytes and offset must also be 4 bytes aligned */
		if ((bytes == 4U) && ((offset & 0x3U) == 0U)) {
			*val = pci_vdev_read_vcfg(vdev, offset, bytes);
		} else {
			*val = ~0U;
		}
	} else {
		/* ToDo: add cfg_hdr_perm for Type 1 device */
		if (bitmap32_test(((uint16_t)offset) >> 2U, &cfg_hdr_perm.pt_mask)) {
			*val = pci_pdev_read_cfg(vdev->pdev->bdf, offset, bytes);

			/* MSE(Memory Space Enable) bit always be set for an assigned VF */
			if ((vdev->phyfun != NULL) && (offset == PCIR_COMMAND) &&
					(vdev->vpci != vdev->phyfun->vpci)) {
				*val |= PCIM_CMD_MEMEN;
			}
		} else {
			*val = pci_vdev_read_vcfg(vdev, offset, bytes);
		}
	}
	return ret;
}

/**
 * @brief Writes the configuration header of a PCI device.
 *
 * This function is responsible for handling write operations to the configuration
 * header of a PCI virtual device (struct pci_vdev). The configuration header contains
 * various registers that control the behavior and configuration of the device.
 *
 * The function takes four parameters: vdev, offset, bytes, and val. The vdev
 * parameter is a pointer to a structure representing the PCI device. The offset
 * parameter specifies the offset within the configuration header that needs to
 * be written. The bytes parameter indicates the number of bytes to write, and the
 * val parameter is an unsigned integer to store the write value.
 *
 * The function completes the write operation by below steps:
 *     1. The function begins by declaring a local variable ret and initializing it to 0.
 *     This variable will be used to store the return value of the function.
 *
 *     2. The code then checks if the offset being written is equal to PCIR_BIOS and if the
 *     virtual device has a specific quirk called quirk_ptdev. If both conditions are true,
 *     the function sets ret to -ENODEV. This is a specific behavior for the PCIR_BIOS offset
 *     when the quirk_ptdev is present.
 *
 *     3. Next, the code checks if the write access is for a Base Address Register (BAR) by
 *     calling the vbar_access function. A BAR is a region of memory or I/O space that a PCI
 *     device uses to communicate with the system. If the write access is indeed for a BAR
 *     and the offset is aligned to 4 bytes, the code proceeds to call the vdev_pt_write_vbar
 *     function to handle the write operation.
 *
 *     4. If the write access is not for a BAR, the code checks if the offset is equal to
 *     PCIR_COMMAND. This register controls various command and status bits for the device.
 *     If the offset is PCIR_COMMAND, the code reads the current value of the physical device's
 *     command register using the pci_pdev_read_cfg function and stores it in the phys_cmd variable.
 *
 *     4.1. The code then checks if a specific condition is met: if the physical command register
 *     value has both the PCIM_CMD_PORTEN and PCIM_CMD_MEMEN bits cleared (indicating that I/O
 *     and memory spaces are disabled), and the new value being written has either of these bits
 *     set, and the physical device (pdev) needs a BAR restore. If all these conditions are true,
 *     the code calls the pdev_restore_bar function to restore the BARs of the physical device. The
 *     function declares PCIM_SPACE_EN as (PCIM_CMD_PORTEN | PCIM_CMD_MEMEN) to help to do above
 *     works.
 *
 *     4.2. After that, the code checks if the offset is allowed to be written by checking a bitmap
 *     called cfg_hdr_perm.ro_mask. If the offset is not marked as read-only in the bitmap, the
 *     code proceeds to check if the offset is marked as a pass-through register in another bitmap
 *     called cfg_hdr_perm.pt_mask. If the offset is marked as pass-through, the code calls the
 *     pci_pdev_write_cfg function to write the value to the physical device's configuration space.
 *     Otherwise, the code calls the pci_vdev_write_vcfg function to write the value to the virtual
 *     device's configuration space.
 *
 *     4.3. The code then checks if the offset is equal to PCIR_INTERRUPT_LINE, which represents the
 *     interrupt line register. If it is, the code masks the value being written with 0xfU (to keep
 *     only the lower 4 bits) and calls the pci_vdev_write_vcfg function to write the modified value
 *     to the virtual device's configuration space. This is done to emulate the behavior of the
 *     interrupt line register even if the PCI device does not support interrupts.
 *
 *     5. Finally, the function returns the value of ret, which will be 0 unless the access to
 *     PCIR_BIOS was emulated for a device with the quirk_ptdev quirk.
 *
 * @param[in] vdev   The virtual device representing the PCI device.
 * @param[in] offset The offset within the configuration header to read from.
 * @param[in] bytes  The number of bytes to read.
 * @param[in] val    Pointer to store the read value.
 *
 * @return 0 on success, or -ENODEV if it tries to write PCIR_BIOS of QUIRK PT device.
 *
 * @pre vdev != NULL
 * @pre vdev->pdev != NULL
 * @pre offset + bytes < PCI_CFG_HEADER_LENGTH
 */
static int32_t write_cfg_header(struct pci_vdev *vdev,
		uint32_t offset, uint32_t bytes, uint32_t val)
{
	int32_t ret = 0;

	if ((offset == PCIR_BIOS) && is_quirk_ptdev(vdev)) {
		/* the access of PCIR_BIOS is emulated for quirk_ptdev */
		ret = -ENODEV;
	} else if (vbar_access(vdev, offset)) {
		/* bar write access must be 4 bytes and offset must also be 4 bytes aligned */
		if ((bytes == 4U) && ((offset & 0x3U) == 0U)) {
			vdev_pt_write_vbar(vdev, pci_bar_index(offset), val);
		}
	} else {
		if (offset == PCIR_COMMAND) {
#define PCIM_SPACE_EN (PCIM_CMD_PORTEN | PCIM_CMD_MEMEN)
			uint16_t phys_cmd = (uint16_t)pci_pdev_read_cfg(vdev->pdev->bdf, PCIR_COMMAND, 2U);

			/* check whether need to restore BAR because some kind of reset */
			if (((phys_cmd & PCIM_SPACE_EN) == 0U) && ((val & PCIM_SPACE_EN) != 0U) &&
					pdev_need_bar_restore(vdev->pdev)) {
				pdev_restore_bar(vdev->pdev);
			}
		}

		/* ToDo: add cfg_hdr_perm for Type 1 device */
		if (!bitmap32_test(((uint16_t)offset) >> 2U, &cfg_hdr_perm.ro_mask)) {
			if (bitmap32_test(((uint16_t)offset) >> 2U, &cfg_hdr_perm.pt_mask)) {
				pci_pdev_write_cfg(vdev->pdev->bdf, offset, bytes, val);
			} else {
				pci_vdev_write_vcfg(vdev, offset, bytes, val);
			}
		}

		/* According to PCIe Spec, for a RW register bits, If the optional feature
		 * that is associated with the bits is not implemented, the bits are permitted
		 * to be hardwired to 0b. However Zephyr would use INTx Line Register as writable
		 * even this PCI device has no INTx, so emulate INTx Line Register as writable.
		 */
		if (offset == PCIR_INTERRUPT_LINE) {
			pci_vdev_write_vcfg(vdev, offset, bytes, (val & 0xfU));
		}

	}
	return ret;
}

/**
 * @brief Writes the device configuration for a virtual PCI device.
 *
 * This function is responsible for writing the device configuration for a
 * virtual PCI device. It takes the virtual device, offset, number of bytes,
 * and value as parameters.
 *
 * The function first checks if the offset corresponds to the configuration
 * header, and if so, it calls the write_cfg_header() function to handle the
 * write operation.
 *
 * If the offset corresponds to the MSI capability, it calls the write_vmsi_cap_reg()
 * function.
 *
 * If the offset corresponds to the MSI-X capability, it checks if the virtual
 * device is using MSI-X on MSI, and if so, it calls and calls the write_vmsix_cap_reg_on_msi()
 * function. Otherwise, it calls write_pt_vmsix_cap_reg() function.
 *
 * If the offset corresponds to the SR-IOV capability, it calls the write_sriov_cap_reg()
 * function.
 *
 * If none of the above conditions are met, it checks if the offset is different from
 * the previous position in the SR-IOV capability. If so, it checks if the virtual
 * device is a quirk passthrough device, and if so, it returns -ENODEV. If the virtual
 * device is not a quirk passthrough device, and the device is not related to the IGD
 * device or offset is not PCIR_ASLS_CTL(0xfc), it calls the pci_pdev_write_cfg() function
 * to write the configuration to the physical device.
 *
 * Finally, it returns 0 on success. If the virtual device is a quirk passthrough device,
 * it returns -ENODEV.
 *
 * @param[in] vdev   The PCI virtual device.
 * @param[in] offset The offset within the device configuration.
 * @param[in] bytes  The number of bytes to write.
 * @param[in] val    The value to write.
 *
 * @return 0 on success, or -ENODEV if it tries to write QUIRK PT device register.
 *
 * @pre vdev != NULL
 * @pre vdev->pdev != NULL
 */
static int32_t write_pt_dev_cfg(struct pci_vdev *vdev, uint32_t offset,
		uint32_t bytes, uint32_t val)
{
	int32_t ret = 0;

	if (cfg_header_access(offset)) {
		ret = write_cfg_header(vdev, offset, bytes, val);
	} else if (msicap_access(vdev, offset)) {
		write_vmsi_cap_reg(vdev, offset, bytes, val);
	} else if (msixcap_access(vdev, offset)) {
		if (vdev->msix.is_vmsix_on_msi) {
			write_vmsix_cap_reg_on_msi(vdev, offset, bytes, val);
		} else {
			write_pt_vmsix_cap_reg(vdev, offset, bytes, val);
		}
	} else if (sriovcap_access(vdev, offset)) {
		write_sriov_cap_reg(vdev, offset, bytes, val);
	} else {
		if (offset != vdev->pdev->sriov.pre_pos) {
			if (!is_quirk_ptdev(vdev)) {
				if ((vdev->pdev->bdf.value != CONFIG_IGD_SBDF) || (offset != PCIR_ASLS_CTL)) {
					/* passthru to physical device */
					pci_pdev_write_cfg(vdev->pdev->bdf, offset, bytes, val);
				}
			} else {
				ret = -ENODEV;
			}
		}
	}

	return ret;
}

/**
 * @brief Reads the configuration space of a virtual PCI device.
 *
 * This function reads the configuration space of a virtual PCI device
 * specified by the `vdev` parameter. The `offset` parameter specifies
 * the starting offset within the configuration space, and the `bytes`
 * parameter specifies the number of bytes to read. The result of the
 * read operation is stored in the `val` parameter.
 *
 * The function first checks if the offset corresponds to the configuration
 * header, and if so, it calls the `read_cfg_header` function to read the
 * configuration header. If the offset corresponds to the MSI capability,
 * the function calls the `pci_vdev_read_vcfg` function to read the MSI
 * capability register. If the offset corresponds to the MSI-X capability,
 * the function calls the `read_pt_vmsix_cap_reg` function to read the
 * MSI-X capability register. If the offset corresponds to the SR-IOV
 * capability, the function calls the `read_sriov_cap_reg` function to
 * read the SR-IOV capability register.
 *
 * If none of the above conditions are met, the function checks if the
 * offset matches the pre-allocated position for SR-IOV and if SR-IOV
 * is hidden. If so, it calls the `pci_vdev_read_vcfg` function to read
 * the configuration space. If the virtual PCI device is not a quirk
 * passthrough device, the function reads the configuration space of the
 * physical device specified by `vdev->pdev->bdf` using the `pci_pdev_read_cfg`
 * function. If the physical device is an IGD device and the offset
 * corresponds to the ASLS control register, the function calls the
 * `pci_vdev_read_vcfg` function to read the configuration space.
 *
 * If the virtual PCI device is a passthrough device, the function returns
 * an error code of -ENODEV. Otherwise, it returns 0.
 *
 * @param[in]  vdev   Pointer to the virtual PCI device structure.
 * @param[in]  offset Starting offset within the configuration space.
 * @param[in]  bytes  Number of bytes to read.
 * @param[out] val    Pointer to store the result of the read operation.
 *
 * @return       0 on success, -ENODEV if the virtual PCI device is a
 *               passthrough device.
 *
 * @pre vdev != NULL
 * @pre vdev->pdev != NULL
 */
static int32_t read_pt_dev_cfg(struct pci_vdev *vdev, uint32_t offset,
		uint32_t bytes, uint32_t *val)
{
	int32_t ret = 0;

	if (cfg_header_access(offset)) {
		ret = read_cfg_header(vdev, offset, bytes, val);
	} else if (msicap_access(vdev, offset)) {
		*val = pci_vdev_read_vcfg(vdev, offset, bytes);
	} else if (msixcap_access(vdev, offset)) {
		read_pt_vmsix_cap_reg(vdev, offset, bytes, val);
	} else if (sriovcap_access(vdev, offset)) {
		read_sriov_cap_reg(vdev, offset, bytes, val);
	} else {
		if ((offset == vdev->pdev->sriov.pre_pos) && (vdev->pdev->sriov.hide_sriov)) {
			*val = pci_vdev_read_vcfg(vdev, offset, bytes);
		} else if (!is_quirk_ptdev(vdev)) {
			/* passthru to physical device */
			*val = pci_pdev_read_cfg(vdev->pdev->bdf, offset, bytes);
			if ((vdev->pdev->bdf.value == CONFIG_IGD_SBDF) && (offset == PCIR_ASLS_CTL)) {
				*val = pci_vdev_read_vcfg(vdev, offset, bytes);
			}
		} else {
			ret = -ENODEV;
		}
	}

	return ret;
}

/**
 * @brief Instance of the pci_vdev_ops structure to store function pointers for
 * operations on a passthrough PCI virtual device.
 *
 * This instance is used when a passthrough PCI device is initialized, deinitialized,
 * or when its configuration is read or written.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
static const struct pci_vdev_ops pci_pt_dev_ops = {
    .init_vdev = vpci_init_pt_dev, /**< Function to initialize a passthrough PCI virtual device. */
    .deinit_vdev = vpci_deinit_pt_dev, /**< Function to deinitialize a passthrough PCI virtual device. */
    .write_vdev_cfg = write_pt_dev_cfg, /**< Function to write the configuration register. */
    .read_vdev_cfg = read_pt_dev_cfg, /**< Function to read the configuration register. */
};

/**
 * @brief Read the configuration space of a virtual PCI device.
 *
 * This function reads the configuration space of a virtual PCI device specified
 * by the bus, device, and function (BDF) address. It retrieves the value at the
 * specified offset and stores it in the provided pointer `val`.
 *
 * The function takes several parameters: a pointer to a structure acrn_vpci, a
 * union pci_bdf named bdf, and three uint32_t variables named offset, bytes, and
 * a pointer val to store the value read out.
 *
 * Inside the function, there is a local variable ret initialized to 0. The function
 * also has a local variable vdev of type struct pci_vdev.
 *
 * The function begins by acquiring a spinlock using the spinlock_obtain function on
 * the vpci structure. This suggests that the function is performing some kind of
 * synchronization to ensure thread safety.
 *
 * Next, the function calls the find_available_vdev function, passing in the vpci
 * structure and the bdf parameter. If the returned value is not NULL, it means that
 * a valid virtual device (vdev) is found, and the function proceeds to call the
 * read_vdev_cfg function on the vdev object. This suggests that the function is reading
 * the configuration of a virtual device.
 *
 * If the vdev is NULL, it means that no virtual device is found. In this case, the
 * function checks if the current context is a post-launched virtual machine by calling
 * the is_postlaunched_vm function on the vpci2vm function. If it is a post-launched VM,
 * the function sets ret to -ENODEV, indicating that the device is not available.
 *
 * If the current context is not a post-launched VM, the function checks if the bdf
 * corresponds to a hidden platform device by calling the is_plat_hidden_pdev function.
 * If it is a hidden platform device, the function reads the configuration using the
 * pci_pdev_read_cfg function and assigns the result to the val parameter.
 *
 * Finally, if none of the above conditions are met, the function does nothing.
 *
 * After the control flow logic, the function releases the spinlock using the spinlock_release
 * function and returns the value of ret.
 *
 * @param[in]  vpci   The pointer to the acrn_vpci structure representing the virtual PCI
 *                    controller.
 * @param[in]  bdf    The BDF address of the PCI device to read from.
 * @param[in]  offset The offset within the configuration space to read from.
 * @param[in]  bytes  The number of bytes to read.
 * @param[out] val    The pointer to store the read value.
 *
 * @return 0 on success, or -ENODEV if the vdev cannot be found in post-launched VM.
 *
 * @pre vpci != NULL
 */
static int32_t vpci_read_cfg(struct acrn_vpci *vpci, union pci_bdf bdf,
	uint32_t offset, uint32_t bytes, uint32_t *val)
{
	int32_t ret = 0;
	struct pci_vdev *vdev;

	spinlock_obtain(&vpci->lock);
	vdev = find_available_vdev(vpci, bdf);
	if (vdev != NULL) {
		ret = vdev->vdev_ops->read_vdev_cfg(vdev, offset, bytes, val);
	} else {
		if (is_postlaunched_vm(vpci2vm(vpci))) {
			ret = -ENODEV;
		} else if (is_plat_hidden_pdev(bdf)) {
			/* expose and pass through platform hidden devices */
			*val = pci_pdev_read_cfg(bdf, offset, bytes);
		} else {
			/* no action: e.g., PCI scan */
		}
	}
	spinlock_release(&vpci->lock);
	return ret;
}

/**
 * @brief Writes a value to the configuration space of a PCI device.
 *
 * This function writes a value to the specified offset in the configuration space
 * of a PCI device identified by the given bus, device, and function (BDF).
 *
 * The vpci_write_cfg function takes several parameters: a pointer to a structure
 * called acrn_vpci, a union pci_bdf object named bdf, and three uint32_t variables
 * named offset, bytes, and val.
 *
 * Inside the function, the first thing is the acquisition of a spinlock using the
 * spinlock_obtain function. This suggests that the code is dealing with concurrent
 * access to a shared resource, and the spinlock is used to synchronize access to
 * that resource.
 *
 * The next step is to find an available virtual PCI device (vdev) by calling the
 * find_available_vdev function, passing in the vpci structure and the bdf object.
 * If a valid vdev is found, the code proceeds to call the write_vdev_cfg function
 * of the vdev object's vdev_ops member, passing in the offset, bytes, and val
 * parameters. This indicates that the function is performing some kind of
 * configuration write operation on the virtual PCI device.
 *
 * If no valid vdev is found, the code checks if the current context is a post-launched
 * virtual machine by calling the is_postlaunched_vm function with the vpci2vm function
 * as its argument. If it is a post-launched VM, the code sets the return value ret to
 * -ENODEV, indicating that the device was not found.
 *
 * If the context is not a post-launched VM, the code checks if the bdf object represents
 * a platform hidden device by calling the is_plat_hidden_pdev function. If it is a hidden
 * device, the code calls the pci_pdev_write_cfg function, passing in the bdf, offset, bytes,
 * and val parameters. This suggests that the function is performing a configuration write
 * operation on a platform hidden device.
 *
 * If none of the above conditions are met, the code prints a log message using the pr_acrnlog
 * function, indicating that the device specified by the bdf object was not found. The log
 * message includes the function name, the bus, device, and function numbers of the bdf object,
 * as well as the offset and val values.
 *
 * Finally, the spinlock is released using the spinlock_release function, and the function
 * returns the value of ret.
 *
 * @param[in] vpci   The pointer to the acrn_vpci structure representing the virtual PCI
 *                   controller.
 * @param[in] bdf    The PCI bus, device, and function (BDF) of the target device.
 * @param[in] offset The offset within the configuration space to write the value.
 * @param[in] bytes  The number of bytes to write.
 * @param[in] val    The value to write to the configuration space.
 *
 * @return 0 on success, or -ENODEV if the vdev cannot be found in post-launched VM.
 *
 * @pre vpci != NULL
 */
static int32_t vpci_write_cfg(struct acrn_vpci *vpci, union pci_bdf bdf,
	uint32_t offset, uint32_t bytes, uint32_t val)
{
	int32_t ret = 0;
	struct pci_vdev *vdev;

	spinlock_obtain(&vpci->lock);
	vdev = find_available_vdev(vpci, bdf);
	if (vdev != NULL) {
		ret = vdev->vdev_ops->write_vdev_cfg(vdev, offset, bytes, val);
	} else {
		if (is_postlaunched_vm(vpci2vm(vpci))) {
			ret = -ENODEV;
		} else if (is_plat_hidden_pdev(bdf)) {
			/* expose and pass through platform hidden devices */
			pci_pdev_write_cfg(bdf, offset, bytes, val);
		} else {
			pr_acrnlog("%s %x:%x.%x not found! off: 0x%x, val: 0x%x\n", __func__,
				bdf.bits.b, bdf.bits.d, bdf.bits.f, offset, val);
		}
	}
	spinlock_release(&vpci->lock);
	return ret;
}

/**
 * @brief Initializes a virtual PCI device.
 *
 * This function initializes a virtual PCI device based on the provided configuration.
 *
 * The function takes several parameters: vpci, which is a pointer to the virtual PCI
 * subsystem; dev_config, which is a pointer to the configuration of the virtual PCI
 * device; and parent_pf_vdev, which is a pointer to the parent physical function virtual
 * device.
 *
 * Inside the function, the first thing is the declaration of a local variable vdev of type
 * struct pci_vdev*, which will hold the initialized virtual PCI device. The function then
 * proceeds to find an available ID for the virtual device by using the ffz64_ex function on
 * the vdev_bitmaps field of the vpci structure. The ffz64_ex function returns the index of
 * the first zero bit in the bitmap, indicating an available ID.
 *
 * If an available ID is found (i.e., id is less than CONFIG_MAX_PCI_DEV_NUM), the function sets the
 * corresponding bit in the vdev_bitmaps to mark it as used. It then proceeds to initialize the vdev
 * structure with various values. The id is assigned to the id field of vdev, and the vpci pointer is
 * assigned to the vpci field of vdev. The bdf field of vdev is set to the value of dev_config->vbdf.value,
 * which represents the bus, device, and function numbers of the virtual device. The pdev field of vdev
 * is set to dev_config->pdev, which represents the physical device associated with the virtual device.
 * The pci_dev_config field of vdev is set to dev_config, which holds the configuration information for
 * the virtual device. Finally, the phyfun field of vdev is set to parent_pf_vdev, which represents the
 * parent physical function virtual device.
 *
 * Next, the function adds the vdev to a hash list using the hlist_add_head function. The hash list is
 * determined by hashing the vbdf.value of dev_config and using it to index into the vdevs_hlist_heads
 * array in the vpci structure. This allows for efficient lookup and retrieval of virtual devices based
 * on their bus, device, and function numbers.
 *
 * After adding the vdev to the hash list, the function checks if the vdev_ops field of dev_config is not
 * NULL. If it is not NULL, it means that a specific set of virtual device operations has been provided
 * for this device. In that case, the vdev_ops field of vdev is set to the provided vdev_ops. Otherwise,
 * if vdev_ops is NULL, the function sets the vdev_ops field of vdev to the default pci_pt_dev_ops operations.
 * This is done with an additional check to ensure that the emu_type of dev_config is PCI_DEV_TYPE_PTDEV and
 * that dev_config->pdev is not NULL. This implies that the virtual device is a pass-through device and
 * requires specific operations for proper configuration.
 *
 * Finally, the function calls the init_vdev function of the vdev_ops associated with the vdev to perform
 * any additional initialization specific to the virtual device.
 *
 * The function then returns the vdev pointer, which represents the initialized virtual PCI device. If no
 * available ID is found, the function returns NULL, indicating that the initialization failed.
 *
 * @param[in,out]  vpci           The pointer to the acrn_vpci structure.
 * @param[in]      dev_config     The pointer to the acrn_vm_pci_dev_config structure containing the device configuration.
 * @param[in]      parent_pf_vdev The pointer to the parent physical function PCI device.
 *
 * @return The pointer to the initialized pci_vdev structure, or NULL if initialization fails.
 *
 * @pre vpci != NULL
 * @pre vpci.pci_vdev_cnt < CONFIG_MAX_PCI_DEV_NUM
 *
 * @post vpci internal pci_vdev list is updated with the new virtual device.
 *
 * @remark The caller of the function vpci_init_vdev should guarantee execution atomically.
 */
struct pci_vdev *vpci_init_vdev(struct acrn_vpci *vpci, struct acrn_vm_pci_dev_config *dev_config, struct pci_vdev *parent_pf_vdev)
{
	struct pci_vdev *vdev = NULL;
	uint32_t id = (uint32_t)ffz64_ex(vpci->vdev_bitmaps, CONFIG_MAX_PCI_DEV_NUM);

	if (id < CONFIG_MAX_PCI_DEV_NUM) {
		bitmap_set_nolock((id & 0x3FU), &vpci->vdev_bitmaps[id >> 6U]);

		vdev = &vpci->pci_vdevs[id];
		vdev->id = id;
		vdev->vpci = vpci;
		vdev->bdf.value = dev_config->vbdf.value;
		vdev->pdev = dev_config->pdev;
		vdev->pci_dev_config = dev_config;
		vdev->phyfun = parent_pf_vdev;

		hlist_add_head(&vdev->link, &vpci->vdevs_hlist_heads[hash64(dev_config->vbdf.value, VDEV_LIST_HASHBITS)]);
		if (dev_config->vdev_ops != NULL) {
			vdev->vdev_ops = dev_config->vdev_ops;
		} else {
			vdev->vdev_ops = &pci_pt_dev_ops;
			ASSERT(dev_config->emu_type == PCI_DEV_TYPE_PTDEV,
				"Only PCI_DEV_TYPE_PTDEV could not configure vdev_ops");
			ASSERT(dev_config->pdev != NULL, "PCI PTDev is not present on platform!");
		}
		vdev->vdev_ops->init_vdev(vdev);
	}
	return vdev;
}

/**
 * @brief Deinitializes a virtual PCI device.
 *
 * This function deinitializes the given virtual PCI device (input parameter)
 * by calling its deinit_vdev() function pointer, removing it from the linked
 * list, clearing its bit in the vdev_bitmaps, and zeroing out its memory.
 *
 * 1. Call a function deinit_vdev through a function pointer vdev_ops stored
 * in the vdev structure.
 *
 * 2. Then call hlist_del(&vdev->link) to remove the vdev structure from a
 * linked list. The link member of the vdev structure is a part of a linked
 * list implementation.
 *
 * 3. Call bitmap_clear_nolock() to clear the vdev from vpci->vdev_bitmaps.
 * It uses the id member of the vdev structure to determine the bit position
 * to clear.
 *
 * 4. Finally, reset the memory of the vdev structure to zero. This effectively
 * clears all the data stored in the structure.
 *
 * @param[in,out] vdev The virtual PCI device to deinitialize.
 *
 * @return N/A
 *
 * @pre vpci != NULL
 * @pre vdev->vpci != NULL
 */
void vpci_deinit_vdev(struct pci_vdev *vdev)
{
	vdev->vdev_ops->deinit_vdev(vdev);

	hlist_del(&vdev->link);
	bitmap_clear_nolock((vdev->id & 0x3FU), &vdev->vpci->vdev_bitmaps[vdev->id >> 6U]);
	memset(vdev, 0U, sizeof(struct pci_vdev));
}

/**
 * @brief Initializes the virtual devices (vdevs) for a given ACRN VM.
 *
 * This function initializes the vdevs for a specified ACRN VM (input parameter)
 * by iterating through the PCI devices defined in the VM configuration. If the
 * vBDF (virtual Bus:Device:Function) of a vdev is unassigned, it will be created
 * by a hypercall. The function also checks the PIO (Programmed I/O) BARs
 * (Base Address Registers) for each vdev.
 *
 * 1. The function begins by declaring some variables, including idx, vdev, vpci,
 * and vm_config. These variables will be used throughout the function to iterate
 * over the PCI devices and store relevant information.
 *
 * 2. The vm_config variable is assigned the configuration of the virtual machine
 * (vm) by calling the get_vm_config function and passing the corresponding vm_id.
 * This configuration contains information about the PCI devices associated with
 * the virtual machine.
 *
 * 3. The function then initializes a variable ret to 0. This variable will be used
 * to track the success or failure of the initialization process.
 *
 * 4. Next, a for loop is used to iterate over the PCI devices specified in the
 * vm_config. The loop runs from idx = 0 to idx < vm_config->pci_dev_num, which means
 * it will iterate over each PCI device in the configuration.
 *
 * 5. Inside the loop, there is an if statement that checks whether the virtual machine
 * is not post-launched or if the vBDF (virtual Bus, Device, Function) of the PCI device
 * is already assigned. If any condition in the if statement is met, the function calls
 * vpci_init_vdev to initialize the virtual PCI device. This function takes the vpci object,
 * the PCI device configuration (&vm_config->pci_devs[idx]), and a NULL parameter. The
 * return value of this function is assigned to the vdev variable.
 *
 * 7. If the vdev variable is NULL, it means that the initialization of the virtual PCI
 * device failed. In this case, an error message is printed using pr_err and the ret is set
 * to -ENODEV. The loop is broken.
 *
 * 8. If the initialization is successful, the function calls check_pt_dev_pio_bars to check
 * the PIO (Programmed I/O) bars of the PCI device. The check_pt_dev_pio_bars function takes
 * the pci_vdevs array of the vpci object as a parameter. If the return value of this function
 * is non-zero (-EIO), it means that there was an error in checking the PIO bars, and the loop
 * is broken.
 *
 * 9. Finally, the function returns the value of ret, which indicates the success or failure
 * of the initialization process. If ret is 0, it means that all the virtual PCI devices were
 * successfully initialized.
 *
 * @param[in,out] vm The ACRN VM for which the vdevs are to be initialized.
 *
 * @return 0 on success, or an error code on failure.
 *
 * @retval 0	   All virtual PCI devices are successfully initialized.
 * @retval -ENODEV The initialization of the virtual PCI device fails.
 * @retval -EIO    PCI device's PIO BAR isn't identical mapping of host address and guest VM address.
 *
 * @pre vm != NULL
 */
static int32_t vpci_init_vdevs(struct acrn_vm *vm)
{
	uint16_t idx;
	struct pci_vdev *vdev;
	struct acrn_vpci *vpci = &(vm->vpci);
	const struct acrn_vm_config *vm_config = get_vm_config(vpci2vm(vpci)->vm_id);
	int32_t ret = 0;

	for (idx = 0U; idx < vm_config->pci_dev_num; idx++) {
		/* the vdev whose vBDF is unassigned will be created by hypercall */
		if ((!is_postlaunched_vm(vm)) || (vm_config->pci_devs[idx].vbdf.value != UNASSIGNED_VBDF)) {
			vdev = vpci_init_vdev(vpci, &vm_config->pci_devs[idx], NULL);
			if (vdev == NULL) {
				pr_err("%s: failed to initialize vpci, increase MAX_PCI_DEV_NUM in scenario!\n", __func__);
				/* TODO: yisun1 - set ret to -ENODEV here */
				break;
			}
			ret = check_pt_dev_pio_bars(&vpci->pci_vdevs[idx]);
			if (ret != 0) {
				break;
			}
		}
	}

	return ret;
}

/**
 * @brief Assigns a PCI device to a virtual machine.
 *
 * The function takes two parameters: a pointer to the target VM (tgt_vm) and a
 * pointer to the PCI device (pcidev) that needs to be assigned.
 *
 * Inside the function, several variables are declared, including ret (used to
 * store the return value), idx (used as a loop counter), and pointers to various
 * data structures related to PCI devices and VMs.
 *
 * The function begins by extracting the physical bus, device, and function (BDF)
 * of the PCI device from the pcidev structure. It then retrieves the service VM
 * (a VM that provides services to other VMs) using the get_service_vm() function.
 *
 * Next, a spinlock is obtained on the service VM's VPCI (Virtual PCI) lock to
 * ensure exclusive access to the VPCI data structures. The function then searches
 * for a virtual device (vdev_in_service_vm) in the service VM's VPCI using the
 * extracted BDF. It checks if the vdev_in_service_vm exists, is owned by the
 * service VM itself, the corresponding physical device exists, is not a host bridge
 * or a bridge. If all conditions are met, it goes to next step. Otherwise, an
 * error message is printed, the ret is set to -ENODEV and exit the function with spinlock
 * released.
 *
 * The function chekcs if the virtual device supports either FLR (Function Level Reset)
 * or PM (Power Management) reset. If any of these conditions are not met, a fatal error
 * message is printed by calling pr_fatal.
 *
 * If the conditions are met, the function restores the base address registers (BARs)
 * of the vdev_in_service_vm device by calling pdev_restore_bar. Then, it deinitializes
 * it by calling its deinit_vdev function.
 *
 * Next, the function gets the target VM's VPCI (vpci) object and obtains a spinlock
 * on it. It then initializes a new virtual device (vdev) in the target VM's VPCI
 * using the configuration and physical function of the vdev_in_service_vm by calling
 * vpci_init_vdev.
 *
 * If the vpci_init_vdev fails to initialize the vdev, the function prints a fatal error
 * and sets ret to -EFAULT. Otherwise, the function sets the interrupt line and interrupt
 * pin of the vdev based on the values from the pcidev structure. If it is a VF assigned
 * to VM, it copies vdev_in_service_vm BAR value to the vdev. If the MSIX is supported,
 * it copies the base address to vdev->msix.mmio_hpa and size to vdev->msix.mmio_size.
 * It also copies the BAR values from the pcidev structure to the vdev.
 *
 * After that, the function checks the PIO (Port Input/Output) BARs of the vdev for any
 * conflicts with other devices. If there are no conflicts, the function sets the appropriate
 * flags and virtual BDF (Bus, Device, Function) for the vdev. It then removes the vdev from
 * the old hash list, and adds it back to the hash list vpci->vdevs_hlist_heads. Finally, it sets
 * the parent_user field of vdev to vdev_in_service_vm and user field of the vdev_in_service_vm
 * to point to the vdev.
 *
 * If there are conflicts with the PIO BARs, the function deinitializes the vdev and reinitializes
 * the vdev_in_service_vm.
 *
 * Finally, the spinlock on the service VM's VPCI is released, and the function returns the
 * value of ret.
 *
 * @param[in] tgt_vm The target virtual machine to assign the PCI device to.
 * @param[in] pcidev The PCI device to be assigned.
 *
 * @return 0 on success, or an error code on failure.
 * @retval 0       PCI device is assigned to target VM successfully.
 * @retval -EIO    PCI device's PIO BAR isn't identical mapping of host address and guest VM address.
 * @retval -ENODEV PCI device is not found.
 *
 * @pre tgt_vm != NULL
 * @pre pcidev != NULL
 */
int32_t vpci_assign_pcidev(struct acrn_vm *tgt_vm, struct acrn_pcidev *pcidev)
{
	int32_t ret = 0;
	uint32_t idx;
	struct pci_vdev *vdev_in_service_vm, *vdev;
	struct acrn_vpci *vpci;
	union pci_bdf bdf;
	struct acrn_vm *service_vm;

	bdf.value = pcidev->phys_bdf;
	service_vm = get_service_vm();
	spinlock_obtain(&service_vm->vpci.lock);
	vdev_in_service_vm = pci_find_vdev(&service_vm->vpci, bdf);
	if ((vdev_in_service_vm != NULL) && (vdev_in_service_vm->user == vdev_in_service_vm) &&
			(vdev_in_service_vm->pdev != NULL) &&
			!is_host_bridge(vdev_in_service_vm->pdev) && !is_bridge(vdev_in_service_vm->pdev)) {

		/* ToDo: Each PT device must support one type reset */
		if (!vdev_in_service_vm->pdev->has_pm_reset && !vdev_in_service_vm->pdev->has_flr &&
				!vdev_in_service_vm->pdev->has_af_flr) {
			pr_fatal("%s %x:%x.%x not support FLR or not support PM reset\n",
				__func__, bdf.bits.b,  bdf.bits.d,  bdf.bits.f);
		} else {
			/* DM will reset this device before assigning it */
			pdev_restore_bar(vdev_in_service_vm->pdev);
		}

		vdev_in_service_vm->vdev_ops->deinit_vdev(vdev_in_service_vm);

		vpci = &(tgt_vm->vpci);

		spinlock_obtain(&tgt_vm->vpci.lock);
		vdev = vpci_init_vdev(vpci, vdev_in_service_vm->pci_dev_config, vdev_in_service_vm->phyfun);
		if (vdev != NULL) {
			pci_vdev_write_vcfg(vdev, PCIR_INTERRUPT_LINE, 1U, pcidev->intr_line);
			pci_vdev_write_vcfg(vdev, PCIR_INTERRUPT_PIN, 1U, pcidev->intr_pin);
			for (idx = 0U; idx < vdev->nr_bars; idx++) {
				/* VF is assigned to a User VM */
				if (vdev->phyfun != NULL) {
					vdev->vbars[idx] = vdev_in_service_vm->vbars[idx];
					if (has_msix_cap(vdev) && (idx == vdev->msix.table_bar)) {
						vdev->msix.mmio_hpa = vdev->vbars[idx].base_hpa;
						vdev->msix.mmio_size = vdev->vbars[idx].size;
					}
				}
				pci_vdev_write_vbar(vdev, idx, pcidev->bar[idx]);
			}

			ret = check_pt_dev_pio_bars(vdev);

			if (ret == 0) {
				vdev->flags |= pcidev->type;
				vdev->bdf.value = pcidev->virt_bdf;
				/*We should re-add the vdev to hashlist since its vbdf has changed */
				hlist_del(&vdev->link);
				hlist_add_head(&vdev->link, &vpci->vdevs_hlist_heads[hash64(vdev->bdf.value, VDEV_LIST_HASHBITS)]);
				vdev->parent_user = vdev_in_service_vm;
				vdev_in_service_vm->user = vdev;
			} else {
				vdev->vdev_ops->deinit_vdev(vdev);
				vdev_in_service_vm->vdev_ops->init_vdev(vdev_in_service_vm);
			}
		} else {
			pr_fatal("%s, Failed to initialize PCI device %x:%x.%x for vm [%d]\n", __func__,
				pcidev->phys_bdf >> 8U, (pcidev->phys_bdf >> 3U) & 0x1fU, pcidev->phys_bdf & 0x7U,
				tgt_vm->vm_id);
			ret = -EFAULT;
		}
		spinlock_release(&tgt_vm->vpci.lock);
	} else {
		pr_fatal("%s, can't find PCI device %x:%x.%x for vm[%d] %x:%x.%x\n", __func__,
			pcidev->phys_bdf >> 8U, (pcidev->phys_bdf >> 3U) & 0x1fU, pcidev->phys_bdf & 0x7U,
			tgt_vm->vm_id,
			pcidev->virt_bdf >> 8U, (pcidev->virt_bdf >> 3U) & 0x1fU, pcidev->virt_bdf & 0x7U);
		ret = -ENODEV;
	}
	spinlock_release(&service_vm->vpci.lock);

	return ret;
}

/**
 * @brief Deassigns a PCI device from a virtual machine.
 *
 * The purpose of this function is to deassign a PCI device from a specific virtual
 * machine (tgt_vm). It takes two parameters: tgt_vm, which represents the target
 * virtual machine, and pcidev, which represents the PCI device to be deassigned.
 *
 * Inside the function, several local variables are declared firstly. These variables
 * include ret of type int32_t to store the return value, parent_vdev and vdev of type
 * struct pci_vdev* to represent the parent and child virtual devices, and vpci of type
 * struct acrn_vpci* to represent the virtual PCI subsystem.
 *
 * The function starts by extracting the virtual (Bus, Device, Function) values from the
 * pcidev structure using a union called bdf. The BDF values uniquely identify a PCI device.
 *
 * Next, the function calls pci_find_vdev to find the virtual device (vdev) associated with
 * the provided BDF values within the target virtual machine's PCI subsystem (tgt_vm->vpci).
 * If the virtual device is found, the virtual device user is itself, the physical device
 * of the virtual device exists and the physical device's BDF equals to the input pcidev's
 * physical BDF, all these conditions are met, the function proceeds with the deassignment
 * process.
 *
 * Inside the conditional block, the function first obtains a spin lock on the virtual PCI
 * subsystem (vpci->lock). This lock ensures that the deassignment process is synchronized
 * and prevents concurrent access to the PCI controller.
 *
 * Then, the function calls vpci_deinit_vdev to deinitialize the virtual device (vdev). This
 * function is responsible for cleaning up any resources associated with the virtual device.
 *
 * After releasing the spin lock, the function checks if there is a parent virtual device
 * (parent_vdev). If a parent virtual device exists, it obtains a spin lock on its PCI subsystem,
 * calls init_vdev to initialize the parent virtual device, and releases the spin lock. This
 * step ensures that the parent virtual device is properly reinitialized after the deassignment.
 *
 * If the conditions in the initial conditional block are not met, indicating that the virtual
 * device or its associated physical device was not found, the function logs an error message
 * using pr_fatal and sets the return value ret to -ENODEV, indicating a device not found error.
 *
 * Finally, the function returns the value of ret, which will be 0 if the deassignment process
 * was successful or -ENODEV if there was an error.
 *
 * @param[in] tgt_vm The target virtual machine from which to deassign the PCI device.
 * @param[in] pcidev The PCI device to be deassigned.
 *
 * @return 0 on success, or a negative error code on failure.
 *
 * @retval 0       PCI device is deassigned from target VM successfully.
 * @retval -ENODEV PCI device is not found.
 *
 * @pre tgt_vm != NULL
 * @pre pcidev != NULL
 */
int32_t vpci_deassign_pcidev(struct acrn_vm *tgt_vm, struct acrn_pcidev *pcidev)
{
	int32_t ret = 0;
	struct pci_vdev *parent_vdev, *vdev;
	struct acrn_vpci *vpci;
	union pci_bdf bdf;

	bdf.value = pcidev->virt_bdf;
	vdev = pci_find_vdev(&tgt_vm->vpci, bdf);
	if ((vdev != NULL) && (vdev->user == vdev) && (vdev->pdev != NULL) &&
			(vdev->pdev->bdf.value == pcidev->phys_bdf)) {
		vpci = vdev->vpci;
		parent_vdev = vdev->parent_user;

		spinlock_obtain(&vpci->lock);
		vpci_deinit_vdev(vdev);
		spinlock_release(&vpci->lock);

		if (parent_vdev != NULL) {
			spinlock_obtain(&parent_vdev->vpci->lock);
			parent_vdev->vdev_ops->init_vdev(parent_vdev);
			spinlock_release(&parent_vdev->vpci->lock);
		}
	} else {
		pr_fatal("%s, can't find PCI device %x:%x.%x for vm[%d] %x:%x.%x\n", __func__,
			pcidev->phys_bdf >> 8U, (pcidev->phys_bdf >> 3U) & 0x1fU, pcidev->phys_bdf & 0x7U,
			tgt_vm->vm_id,
			pcidev->virt_bdf >> 8U, (pcidev->virt_bdf >> 3U) & 0x1fU, pcidev->virt_bdf & 0x7U);
		ret = -ENODEV;
	}

	return ret;
}

/**
 * @brief Updates a single virtual base address register (vBAR) for a virtual PCI device.
 *
 * This function updates the value of a specific vBAR for a given virtual PCI device.
 * It also provides callbacks for mapping and unmapping the vBAR to/from the guest
 * physical address space.
 *
 * The function takes in several parameters: a pointer to a pci_vdev structure named
 * vdev, two uint32_t variables named bar_idx and val, and two function pointers named
 * map_cb and unmap_cb.
 *
 * Inside the function, there is a declaration of a pci_vbar structure pointer named
 * vbar, which is initialized to the address of the vbars array element at index bar_idx
 * within the vdev structure. The vbars array is a member of the pci_vdev structure.
 *
 * Next, there is a conditional statement that checks if the is_mem64hi flag of the vbar
 * structure is set. If it is, the update_idx variable is decremented by 1. This suggests
 * that the is_mem64hi flag indicates the high part of 64 bits MMIO bar.
 *
 * After that, the unmap_cb function is called with the vdev and update_idx parameters. This
 * function is responsible for unmapping the previously mapped memory region associated with
 * the update_idx.
 *
 * Following the unmapping step, the pci_vdev_write_vbar function is called with the vdev,
 * bar_idx, and val parameters. This function is responsible for updating the value of the
 * specified vbar with the provided val.
 *
 * Finally, there is another conditional statement that checks if the map_cb function pointer
 * is not NULL and if the base_gpa member of the vbars array element at index update_idx is
 * not zero. If both conditions are true, the map_cb function is called with the vdev and
 * update_idx parameters. This suggests that the map_cb function is responsible for mapping
 * the memory region associated with the update_idx.
 *
 * @param[in,out] vdev     Pointer to the PCI device structure.
 * @param[in]     bar_idx  Index of the vBAR to be updated.
 * @param[in]     val      New value to be written to the vBAR.
 * @param[in]     map_cb   Callback function for mapping the vBAR to the guest physical address space.
 * @param[in]     unmap_cb Callback function for unmapping the vBAR from the guest physical address space.
 *
 * @pre vdev != NULL
 * @pre unmap_cb != NULL
 *
 * @post vdev->vbars[bar_idx] (or vdev->vbars[bar_idx - 1] if vdev->vbars[bar_idx]->is_mem64hi == TRUE) is updated.
 */
void vpci_update_one_vbar(struct pci_vdev *vdev, uint32_t bar_idx, uint32_t val,
		map_pcibar map_cb, unmap_pcibar unmap_cb)
{
	struct pci_vbar *vbar = &vdev->vbars[bar_idx];
	uint32_t update_idx = bar_idx;

	if (vbar->is_mem64hi) {
		update_idx -= 1U;
	}
	unmap_cb(vdev, update_idx);
	pci_vdev_write_vbar(vdev, bar_idx, val);
	if ((map_cb != NULL) && (vdev->vbars[update_idx].base_gpa != 0UL)) {
		map_cb(vdev, update_idx);
	}
}

/**
 * @brief Adds a capability to a PCI virtual device.
 *
 * This function is responsible for adding a capability to a PCI device.
 *
 * The function takes in three parameters: a pointer to a pci_vdev structure named
 * vdev, a pointer to a uint8_t named capdata which represents the capability data
 * to add for the PCI device, and a uint8_t parameter named caplen which means the
 * length of capdata.
 *
 * The function begins by defining a macro CAP_START_OFFSET which represents the
 * starting offset of the capability within the PCI configuration space. It is
 * set to PCI_CFG_HEADER_LENGTH.
 *
 * Next, the function declares and initializes some variables. capoff represents
 * the offset of the current capability, reallen stores the length of the capability
 * rounded up to the nearest multiple of 4 (to ensure dword alignment), sts holds
 * the value of the PCI device's status register, and ret is the return value of
 * the function, initialized to 0.
 *
 * The code then reads the value of the status register using the pci_vdev_read_vcfg
 * function, passing the PCI device (vdev), the offset of the status register (PCIR_STATUS),
 * and the size of the register (2 bytes). The result is stored in the sts variable.
 *
 * The next block of code checks if the capability is already present in the device. If
 * the PCIM_STATUS_CAPPRESENT flag is not set in the sts variable, it means that no
 * capability is present, and the capoff is set to the CAP_START_OFFSET. Otherwise, the
 * capoff is set to the free_capoff value stored in the vdev structure.
 *
 * The code then checks if there is enough space in the PCI configuration space to add
 * the capability. It calculates the sum of the capoff and reallen and compares it to
 * the maximum size of the PCI configuration space (PCI_CONFIG_SPACE_SIZE). If there
 * is enough space, the code proceeds to add the capability.
 *
 * If PCIM_STATUS_CAPPRESENT flag is not set in the sts variable, it writes the
 * CAP_START_OFFSET to the PCIR_CAP_PTR register using the pci_vdev_write_vcfg function.
 * It also sets sts and the PCIM_STATUS_CAPPRESENT flag together into the status register.
 *
 * If a previous capability is present, the code writes the capoff value (vdev->free_capoff)
 * to the next capability pointer field of the previous capability using the vdev->prev_capoff
 * offset.
 *
 * Next, the code copies the capability data from the capdata buffer to the PCI configuration
 * space at the capoff offset using the memcpy_s function. It also updates the next capability
 * pointer field by writing 0 to the capoff + 1 offset.
 *
 * Finally, the function updates the prev_capoff and free_capoff values in the vdev structure
 * and sets the ret variable to the capoff value.
 *
 * @param[in,out] vdev    Pointer to the PCI virtual device structure.
 * @param[in]     capdata Pointer to the capability data.
 * @param[in]     caplen  Length of the capability data.
 *
 * @return The offset of the added capability, or 0 if the capability could not be added.
 *
 * @pre vdev != NULL
 * @pre vdev->vpci != NULL
 */
uint32_t vpci_add_capability(struct pci_vdev *vdev, uint8_t *capdata, uint8_t caplen)
{
#define CAP_START_OFFSET PCI_CFG_HEADER_LENGTH

	uint8_t capoff, reallen;
	uint32_t sts;
	uint32_t ret = 0U;

	reallen = roundup(caplen, 4U); /* dword aligned */

	sts = pci_vdev_read_vcfg(vdev, PCIR_STATUS, 2U);
	if ((sts & PCIM_STATUS_CAPPRESENT) == 0U) {
		capoff = CAP_START_OFFSET;
	} else {
		capoff = vdev->free_capoff;
	}

	/* Check if we have enough space */
	if (((uint16_t)capoff + reallen) <= PCI_CONFIG_SPACE_SIZE) {
		/* Set the previous capability pointer */
		if ((sts & PCIM_STATUS_CAPPRESENT) == 0U) {
			pci_vdev_write_vcfg(vdev, PCIR_CAP_PTR, 1U, capoff);
			pci_vdev_write_vcfg(vdev, PCIR_STATUS, 2U, sts|PCIM_STATUS_CAPPRESENT);
		} else {
			pci_vdev_write_vcfg(vdev, vdev->prev_capoff + 1U, 1U, capoff);
		}

		/* Copy the capability */
		(void)memcpy_s((void *)&vdev->cfgdata.data_8[capoff], caplen, (void *)capdata, caplen);

		/* Set the next capability pointer */
		pci_vdev_write_vcfg(vdev, capoff + 1U, 1U, 0U);

		vdev->prev_capoff = capoff;
		vdev->free_capoff = capoff + reallen;
		ret = capoff;
	}

	return ret;
}

/**
 * @brief Check if the Virtual Machine Serial Interrupt (vMSI-X) is enabled for a given virtual PCI device.
 *
 * The function checks whether the vMSI-X is enabled for a given virtual PCI device. It takes a
 * pointer to a struct pci_vdev as its parameter.
 *
 * First, the function checks if the msix.capoff member of the vdev structure is not zero. This
 * suggests that the vdev structure contains information about the capabilities of the PCI
 * device, and specifically, the presence of the MSI-X (Message Signaled Interrupts eXtended)
 * capability.
 *
 * If the capoff is not zero, the function proceeds to read the value of the PCIR_MSIX_CTRL
 * register from the PCI configuration space using the pci_vdev_read_vcfg function. The
 * PCIR_MSIX_CTRL is the offset of the MSI-X control register within the PCI configuration
 * space.
 *
 * The value read from the PCIR_MSIX_CTRL register is stored in the msgctrl variable.
 *
 * Next, the function checks two conditions using bitwise operations. First, it checks if the
 * PCIM_MSIXCTRL_MSIX_ENABLE bit is set in the msgctrl value. This bit indicates whether the
 * MSI-X feature is enabled for the device. If it is enabled, the function proceeds to check
 * the second condition.
 *
 * The second condition checks if the PCIM_MSIXCTRL_FUNCTION_MASK bits are not set in the
 * msgctrl value. These bits represent the function mask, which determines which functions of
 * the device are allowed to use MSI-X interrupts. If the function mask is zero, it means that
 * all functions are allowed to use MSI-X interrupts.
 *
 * If both conditions are met, the function sets the ret variable to true, indicating that the
 * vMSI-X is enabled for the PCI device. Otherwise, ret remains false.
 *
 * Finally, the function returns the value of ret, which represents whether the vMSI-X is enabled
 * or not for the given PCI device.
 *
 * @param[in] vdev The PCI device for which to check vMSI-X enablement.
 *
 * @return True if vMSI-X is enabled, False otherwise.
 *
 * @pre vdev != NULL
 */
bool vpci_vmsix_enabled(const struct pci_vdev *vdev)
{
	uint32_t msgctrl;
	bool ret = false;

	if (vdev->msix.capoff != 0U) {
		msgctrl = pci_vdev_read_vcfg(vdev, vdev->msix.capoff + PCIR_MSIX_CTRL, 2U);
		if (((msgctrl & PCIM_MSIXCTRL_MSIX_ENABLE) != 0U) &&
			((msgctrl & PCIM_MSIXCTRL_FUNCTION_MASK) == 0U)) {
			ret = true;
		}
	}
	return ret;
}
