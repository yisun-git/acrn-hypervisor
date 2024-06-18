/*
 * Copyright (C) 2018-2022 Intel Corporation.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <types.h>
#include <errno.h>
#include <asm/vmx.h>
#include <asm/guest/guest_memory.h>
#include <asm/guest/vcpu.h>
#include <asm/guest/vm.h>
#include <asm/guest/vmcs.h>
#include <asm/mmu.h>
#include <asm/guest/ept.h>
#include <logmsg.h>

/**
 * @file
 * @brief This file declares APIs used to handle guest memory.
 *
 * This file declares external APIs to handle guest memory. In addition, it defines some helper functions
 * that are used to implement those external APIs.
 */

/**
 * @brief Data structure to store information about a page walk operation.
 *
 * Each member provides specific details about the page walk operation.
 *
 * It is supposed to be used when the hypervisor performs a page walk operation.
 *
 * @consistency N/A
 * @alignment N/A
 *
 * @remark N/A
 */
struct page_walk_info {
    uint64_t top_entry; /**< Top level paging structure entry. */
    uint32_t level; /**< The level of the page table. */
    uint32_t width; /**< The width of the page table entry. */
    bool is_user_mode_access; /**< Indicates if the access is in user mode. */
    bool is_write_access; /**< Indicates if the access is a write operation. */
    bool is_inst_fetch; /**< Indicates if the access is an instruction fetch. */
    /**
     * @brief Indicates the Page Size Extension (PSE) status.
     *
     * It's true for PAE/4-level paging and represents CR4.PSE for 32bit paging.
     */
    bool pse;
    bool wp; /**< Represents CR0.WP. */
    bool nxe; /**< Represents MSR_IA32_EFER_NXE_BIT. */
    bool is_smap_on; /**< Indicates if the Supervisor Mode Access Prevention (SMAP) is on. */
    bool is_smep_on; /**< Indicates if the Supervisor Mode Execution Prevention (SMEP) is on. */
};

/**
 * @brief Get the paging mode of a virtual CPU.
 *
 * This function determines the paging mode of a virtual CPU based on its configuration.
 *
 * The function takes a pointer to a structure called acrn_vcpu as its parameter and
 * returns an enumeration value representing the paging mode.
 *
 * The function begins by initializing a variable ret with the value PAGING_MODE_0_LEVEL,
 * which corresponds to the non-paging mode. This means that if paging is not enabled for
 * the virtual CPU, the function will return the default non-paging mode.
 *
 * The code then checks if paging is enabled for the virtual CPU by calling the
 * is_paging_enabled function. If paging is enabled, the code proceeds to further checks.
 *
 * The next check is for the presence of Physical Address Extension (PAE) support in the
 * virtual CPU. PAE allows the CPU to access more than 4GB of physical memory in a 32-bit
 * system. If PAE is enabled, the code checks if the virtual CPU is in long mode by calling
 * the is_long_mode function.
 *
 * If the virtual CPU is in long mode, indicating a 64-bit system, the ret variable is set
 * to PAGING_MODE_4_LEVEL, representing 4-level paging. This means that the virtual CPU is
 * using a 4-level page table structure to manage memory.
 *
 * If the virtual CPU is not in long mode but has PAE enabled, the ret variable is set to
 * PAGING_MODE_3_LEVEL, representing PAE paging. This means that the virtual CPU is using a
 * 3-level page table structure.
 *
 * If the virtual CPU does not have PAE enabled, the ret variable is set to PAGING_MODE_2_LEVEL,
 * representing 32-bit paging. This means that the virtual CPU is using a 2-level page table
 * structure.
 *
 * Finally, the function returns the value of the ret variable, indicating the determined
 * paging mode of the virtual CPU.
 *
 * @param[in] vcpu The virtual CPU for which to determine the paging mode.
 *
 * @return The paging mode of the virtual CPU.
 *
 * @retval PAGING_MODE_0_LEVEL: non-paging
 * @retval PAGING_MODE_2_LEVEL: 32-bit paging
 * @retval PAGING_MODE_3_LEVEL: PAE paging
 * @retval PAGING_MODE_4_LEVEL: 4-level paging
 *
 * @pre vcpu != NULL
 */
enum vm_paging_mode get_vcpu_paging_mode(struct acrn_vcpu *vcpu)
{
	enum vm_paging_mode ret = PAGING_MODE_0_LEVEL;    /* non-paging */

	if (is_paging_enabled(vcpu)) {
		if (is_pae(vcpu)) {
			if (is_long_mode(vcpu)) {
				ret = PAGING_MODE_4_LEVEL;    /* 4-level paging */
			} else {
				ret = PAGING_MODE_3_LEVEL;    /* PAE paging */
			}
		} else {
			ret = PAGING_MODE_2_LEVEL;    /* 32-bit paging */
		}
	}

	return ret;
}

/**
 * @brief Converts a guest virtual address (GVA) to a guest physical address (GPA) using page walk information.
 *
 * This function is used to convert a guest virtual address (GVA) to a guest
 * physical address (GPA) using a common page walk.
 *
 * The function takes in several parameters, including a pointer to a struct
 * acrn_vcpu object, a struct page_walk_info object, the GVA, and pointers to
 * variables for the GPA and error code.
 *
 * The function starts by initializing some variables and flags that will be
 * used throughout the translation process. It checks if the page walk level
 * provided in the page_walk_info object is valid. If not, it returns an error
 * code.
 *
 * Next, the function gets the guest page table address from pw_info->top_entry
 * and the page table level. It sets AC flag. Then, it enters a loop that iterates
 * through the page walk levels, starting from the top level. Within each iteration,
 * it performs the necessary operations to translate the GVA to a GPA.
 *
 * First, it masks the address to ensure it aligns with the physical address
 * format. Then, it retrieves the base address of the page table from the guest
 * physical address space by calling gpa2hva function. If the base address is
 * NULL, indicating a fault in the translation process, the function sets the
 * fault flag to 1.
 *
 * The function then calculates the index within the page table based on the
 * current page walk level and the width of the page table entries. It also
 * determines the page size based on the shift value.
 *
 * Depending on the width of the page table entries, the function retrieves the
 * entry from the page table. If the entry does not have the PAGE_PRESENT flag
 * set, indicating that the page is not present in memory, the function sets the
 * fault flag to 1.
 *
 * If there is no fault found and the page is not writable, the function sets the
 * is_page_rw_flags_on flag to false. The translation process also checks if it is
 * a write access, and it is a user mode access or it is super mode access but write
 * protect is enabled. If above conditions are met, the function sets the fault flag
 * to 1.
 *
 * The function then checks for the No-Execute (NX) flag. If the translation
 * process requires an instruction fetch, the guest MSR_IA32_EFER_NXE_BIT value
 * contained in page_walk_info object is set and the NX flag is set in the page
 * table entry, the function sets the fault flag to 1.
 *
 * After that, the function checks the user/supervisor (U/S) mode permissions of
 * the page. If the page is not accessible in user mode and the translation process
 * requires a user mode access, the function sets the fault flag to 1. It also sets
 * the is_user_mode_addr flag to false.
 *
 * If the translation process requires the use of a large page (PSE) and the current
 * page walk level allows it, the function breaks out of the loop. Otherwise, the
 * function then updates the address for the next iteration by assigning the entry
 * value to the addr variable.
 *
 * After the loop, the function checks for the Supervisor Mode Access Prevention (SMAP)
 * and Supervisor Mode Execution Prevention (SMEP) features if there is no fault.
 *
 * If SMAP is enabled and the translation process is not in user mode, but the address
 * being translated is in user mode, the function reads the flag register AC bit, if
 * it is not a write access and AC bit is 0, the function sets the fault flag to 1. If
 * it is a write access, but write protection contained in page_walk_info object is 0
 * and AC bit is 0, the fault is set to 1 too. If it is a write access, the write
 * protection contained in page_walk_info object is 1, the AC bit is 1 but the page is
 * not writable, the fault is set to 1. Furthermore, if the write protection contained
 * in page_walk_info object is 1 but AC bit is 0, the fault is set to 1.
 *
 * Similarly, if SMEP is enabled and the translation process is not in
 * user mode, but the address being translated is in user mode and an instruction
 * fetch is required, the function sets the fault flag to 1.
 *
 * If no faults have occurred up to this point, the function performs the final
 * steps of the translation process. It shifts the entry value to clear the irrelevant
 * bits, and then combines it with the GVA to obtain the GPA. The GPA is stored in the
 * gpa variable.
 *
 * Finally, the function clears the AC flags and checking if any faults occurred. If a
 * fault occurred, the function returns an error code and sets the err_code variable to
 * indicate a page fault.
 *
 * @param[in]  vcpu     The pointer to the virtual CPU structure.
 * @param[in]  pw_info  The pointer to the page walk information structure.
 * @param[in]  gva      The guest virtual address to be converted.
 * @param[out] gpa      The pointer to store the converted guest physical address.
 * @param[out] err_code Pointer to store the error code if an error occurs during the conversion.
 *
 * @return Returns 0 on success, or negative error code for failure.
 *
 * @retval -EINVAL If the level is less than 1.
 * @retval -EFAULT If a fault occurs during the page walk.
 * @retval 0       If the conversion is successful.
 *
 * @pre vcpu != NULL
 * @pre pw_info != NULL
 * @pre gpa != NULL
 * @pre err_code != NULL
 */
static int32_t local_gva2gpa_common(struct acrn_vcpu *vcpu, const struct page_walk_info *pw_info,
	uint64_t gva, uint64_t *gpa, uint32_t *err_code)
{
	uint32_t i;
	uint64_t index;
	uint32_t shift;
	void *base;
	uint64_t entry = 0U;
	uint64_t addr;
	uint64_t page_size = PAGE_SIZE_4K;
	int32_t ret = 0;
	int32_t fault = 0;
	bool is_user_mode_addr = true;
	bool is_page_rw_flags_on = true;

	if (pw_info->level < 1U) {
		ret = -EINVAL;
	} else {
		addr = pw_info->top_entry;
		i = pw_info->level;
		stac();

		while ((i != 0U) && (fault == 0)) {
			i--;

			addr = addr & IA32E_REF_MASK;
			base = gpa2hva(vcpu->vm, addr);
			if (base == NULL) {
				fault = 1;
			} else {
				shift = (i * pw_info->width) + 12U;
				index = (gva >> shift) & ((1UL << pw_info->width) - 1UL);
				page_size = 1UL << shift;

				if (pw_info->width == 10U) {
					uint32_t *base32 = (uint32_t *)base;
					/* 32bit entry */
					entry = (uint64_t)(*(base32 + index));
				} else {
					uint64_t *base64 = (uint64_t *)base;
					entry = *(base64 + index);
				}

				/* check if the entry present */
				if ((entry & PAGE_PRESENT) == 0U) {
					fault = 1;
				}

					/* check for R/W */
				if ((fault == 0) && ((entry & PAGE_RW) == 0U)) {
					if (pw_info->is_write_access  &&
					    (pw_info->is_user_mode_access || pw_info->wp)) {
						/* Case1: Supermode and wp is 1
						 * Case2: Usermode */
						fault = 1;
					}
					is_page_rw_flags_on = false;
				}
			}

			/* check for nx, since for 32-bit paing, the XD bit is
			 * reserved(0), use the same logic as PAE/4-level paging */
			if ((fault == 0) && pw_info->is_inst_fetch && pw_info->nxe &&
				    ((entry & PAGE_NX) != 0U)) {
				fault = 1;
			}

			/* check for U/S */
			if ((fault == 0) && ((entry & PAGE_USER) == 0U)) {
				is_user_mode_addr = false;

				if (pw_info->is_user_mode_access) {
					fault = 1;
				}
			}


			if ((fault == 0) && pw_info->pse &&
				((i > 0U) && ((entry & PAGE_PSE) != 0U))) {
					break;
			}
			addr = entry;
		}

		/* When SMAP/SMEP is on, we only need to apply check when address is
		 * user-mode address.
		 * Also SMAP/SMEP only impact the supervisor-mode access.
		 */
		/* if smap is enabled and supervisor-mode access */
		if ((fault == 0) && pw_info->is_smap_on && (!pw_info->is_user_mode_access) &&
			is_user_mode_addr) {
			bool acflag = ((vcpu_get_rflags(vcpu) & RFLAGS_AC) != 0UL);

			/* read from user mode address, eflags.ac = 0 */
			if ((!pw_info->is_write_access) && (!acflag)) {
				fault = 1;
			} else if (pw_info->is_write_access) {
				/* write to user mode address */

				/* cr0.wp = 0, eflags.ac = 0 */
				if ((!pw_info->wp) && (!acflag)) {
					fault = 1;
				}

				/* cr0.wp = 1, eflags.ac = 1, r/w flag is 0
				 * on any paging structure entry
				 */
				if (pw_info->wp && acflag && (!is_page_rw_flags_on)) {
					fault = 1;
				}

				/* cr0.wp = 1, eflags.ac = 0 */
				if (pw_info->wp && (!acflag)) {
					fault = 1;
				}
			} else {
				/* do nothing */
			}
		}

		/* instruction fetch from user-mode address, smep on */
		if ((fault == 0) && pw_info->is_smep_on && (!pw_info->is_user_mode_access) &&
			is_user_mode_addr && pw_info->is_inst_fetch) {
			fault = 1;
		}

		if (fault == 0) {
			entry >>= shift;
			/* shift left 12bit more and back to clear XD/Prot Key/Ignored bits */
			entry <<= (shift + 12U);
			entry >>= 12U;
			*gpa = entry | (gva & (page_size - 1UL));
		}

		clac();
		if (fault != 0) {
			ret = -EFAULT;
			*err_code |= PAGE_FAULT_P_FLAG;
		}
	}
	return ret;
}

/**
 * @brief Translates a guest virtual address (GVA) to a guest physical address (GPA) using the PAE paging mode.
 *
 * This function is used to convert a guest virtual address (GVA) to a guest
 * physical address (GPA) using Physical Address Extension (PAE) paging. PAE
 * is an extension to the x86 architecture that allows addressing more than
 * 4GB of physical memory.It is typically used when the guest is using PAE
 * paging and there is a need to convert a GVA to a GPA. The function internally
 * calls the get_pae_pdpt_addr, gpa2hva, and local_gva2gpa_common functions,
 * which perform the actual conversion.
 *
 * The function takes several parameters: a pointer to a struct acrn_vcpu object
 * named vcpu, a pointer to a struct page_walk_info object named pw_info, the
 * GVA to be translated (gva), pointers to store the resulting GPA (gpa) and
 * error code (err_code).
 *
 * The function starts by declaring some local variables. It initializes addr
 * with the guest physical address of the Page Directory Pointer Table (PDPT)
 * using the get_pae_pdpt_addr function. Then, it uses gpa2hva to convert the
 * guest physical address to a host virtual address (base) so that hypervisor
 * can access.
 *
 * Next, the function calculates the index into the base array by shifting the
 * GVA right by 30 bits. It then performs a memory access to retrieve the entry
 * at the calculated index. Before accessing the memory, it calls stac(), and
 * after the memory access, it calls clac() to restore.
 *
 * If the entry indicates that the page is present (by checking the PAGE_PRESENT
 * flag), the function sets the level field of pw_info to 2 and updates the top_entry
 * field with the retrieved entry. Finally, it calls local_gva2gpa_common to perform
 * the common translation logic for the given level.
 *
 * The function returns -EFAULT if there was an error during the translation
 * process, otherwise it returns the result of local_gva2gpa_common.
 *
 * @param[in]     vcpu The pointer to the virtual CPU structure.
 * @param[in,out] pw_info The pointer to the page walk information structure.
 * @param[in]     gva The guest virtual address to be converted.
 * @param[out]    gpa The pointer to store the converted guest physical address.
 * @param[out]    err_code Pointer to store the error code if an error occurs during the conversion.
 *
 * @return Returns 0 on success, or -EFAULT if the conversion fails.
 *
 * @retval -EFAULT If the conversion fails.
 * @retval 0       If the conversion is successful.
 *
 * @pre vcpu != NULL
 * @pre pw_info != NULL
 * @pre gpa != NULL
 * @pre err_code != NULL
 */
static int32_t local_gva2gpa_pae(struct acrn_vcpu *vcpu, struct page_walk_info *pw_info,
	uint64_t gva, uint64_t *gpa, uint32_t *err_code)
{
	uint32_t index;
	uint64_t *base;
	uint64_t entry;
	uint64_t addr;
	int32_t ret = -EFAULT;

	addr = get_pae_pdpt_addr(pw_info->top_entry);
	base = (uint64_t *)gpa2hva(vcpu->vm, addr);
	if (base != NULL) {
		index = (uint32_t)gva >> 30U;
		stac();
		entry = base[index];
		clac();

		if ((entry & PAGE_PRESENT) != 0U) {
			pw_info->level = 2U;
			pw_info->top_entry = entry;
			ret = local_gva2gpa_common(vcpu, pw_info, gva, gpa, err_code);
		}
	}

	return ret;
}

/**
 * @brief Converts a guest virtual address (GVA) to a guest physical address (GPA).
 *
 * This function converts a guest virtual address (GVA) to a guest physical address
 * (GPA) using the page walk mechanism. It takes into account the paging mode of the
 * virtual CPU, as well as various control flags and registers.
 *
 * The function takes in several parameters: a pointer to a struct acrn_vcpu object
 * representing a virtual CPU, a GVA, pointers to a GPA and an error code. It returns
 * an int32_t value, which is a status indicator.
 *
 * The first part of the code checks if the gpa and err_code pointers are valid. If
 * either of them is NULL, the function returns an error code -EINVAL. Otherwise, it
 * proceeds with the address translation process.
 *
 * The code initializes the pw_info struct, which holds information related to the page
 * walk process. It sets the top_entry field to the value of the guest control register
 * CR3, which is obtained using the exec_vmread function. The level field is set based
 * on the virtual machine's paging mode. The is_write_access and is_inst_fetch fields
 * are determined by checking specific bits in the err_code parameter.
 *
 * Next, the code determines whether the guest is in user mode by examining the access
 * rights of the guest segment register SS. It uses the DPL (Descriptor Privilege Level)
 * field of the access rights to determine the guest DPL (Descriptor Privilege Level).
 * This information is stored in the is_user_mode_access field of pw_info.
 *
 * The code then sets various flags in the pw_info struct based on the virtual machine's
 * configuration. These flags include pse (Page Size Extension), nxe (No-Execute Enable), wp
 * (Write Protect), is_smap_on (Supervisor Mode Access Prevention), and is_smep_on (Supervisor
 * Mode Execution Prevention).
 *
 * After setting up the pw_info struct, the code clears the PAGE_FAULT_P_FLAG bit in the err_code
 * parameter. This bit indicates a protection violation during the page walk process.
 *
 * The code then performs the actual address translation based on the paging mode. If the paging
 * mode is 4-level, it calls the local_gva2gpa_common function with a width of 9 bits. If the
 * paging mode is 3-level, it calls the local_gva2gpa_pae function with the same width. If the
 * paging mode is 2-level, it sets the width to 10 bits and adjusts the pse and nxe flags accordingly
 * before calling local_gva2gpa_common. If the paging mode is not recognized, it simply assigns
 * the GVA to the GPA.
 *
 * Finally, if the address translation fails with an -EFAULT error code, the code checks if the
 * guest is in user mode. If so, it sets the PAGE_FAULT_US_FLAG bit in the err_code parameter.
 *
 * @param[in]     vcpu     The virtual CPU for which the conversion is performed.
 * @param[in]     gva      The guest virtual address to be converted.
 * @param[out]    gpa      Pointer to store the resulting guest physical address.
 * @param[in,out] err_code Pointer to store the error code, if any.
 *
 * @return 0 on success, or a negative error code on failure.
 *
 * @retval -EFAULT If there is fault during the convert operation.
 * @retval -EINVAL If the provided parameters are invalid.
 * @retval 0       If the convert operation is successful.
 *
 * @pre vcpu != NULL
 * @pre gpa != NULL
 * @pre error_code != NULL
 */
int32_t gva2gpa(struct acrn_vcpu *vcpu, uint64_t gva, uint64_t *gpa,
	uint32_t *err_code)
{
	enum vm_paging_mode pm = get_vcpu_paging_mode(vcpu);
	struct page_walk_info pw_info;
	int32_t ret = 0;

	if ((gpa == NULL) || (err_code == NULL)) {
		ret = -EINVAL;
	} else {
		*gpa = 0UL;

		pw_info.top_entry = exec_vmread(VMX_GUEST_CR3);
		pw_info.level = (uint32_t)pm;
		pw_info.is_write_access = ((*err_code & PAGE_FAULT_WR_FLAG) != 0U);
		pw_info.is_inst_fetch = ((*err_code & PAGE_FAULT_ID_FLAG) != 0U);

		/* SDM vol3 27.3.2
		 * If the segment register was unusable, the base, select and some
		 * bits of access rights are undefined. With the exception of
		 * DPL of SS
		 * and others.
		 * So we use DPL of SS access rights field for guest DPL.
		 */
		pw_info.is_user_mode_access = (((exec_vmread32(VMX_GUEST_SS_ATTR) >> 5U) & 0x3U) == 3U);
		pw_info.pse = true;
		pw_info.nxe = ((vcpu_get_efer(vcpu) & MSR_IA32_EFER_NXE_BIT) != 0UL);
		pw_info.wp = ((vcpu_get_cr0(vcpu) & CR0_WP) != 0UL);
		pw_info.is_smap_on = ((vcpu_get_cr4(vcpu) & CR4_SMAP) != 0UL);
		pw_info.is_smep_on = ((vcpu_get_cr4(vcpu) & CR4_SMEP) != 0UL);

		*err_code &=  ~PAGE_FAULT_P_FLAG;

		if (pm == PAGING_MODE_4_LEVEL) {
			pw_info.width = 9U;
			ret = local_gva2gpa_common(vcpu, &pw_info, gva, gpa, err_code);
		} else if (pm == PAGING_MODE_3_LEVEL) {
			pw_info.width = 9U;
			ret = local_gva2gpa_pae(vcpu, &pw_info, gva, gpa, err_code);
		} else if (pm == PAGING_MODE_2_LEVEL) {
			pw_info.width = 10U;
			pw_info.pse = ((vcpu_get_cr4(vcpu) & CR4_PSE) != 0UL);
			pw_info.nxe = false;
			ret = local_gva2gpa_common(vcpu, &pw_info, gva, gpa, err_code);
		} else {
			*gpa = gva;
		}

		if (ret == -EFAULT) {
			if (pw_info.is_user_mode_access) {
				*err_code |= PAGE_FAULT_US_FLAG;
			}
		}
	}

	return ret;
}

/**
 * @brief Copies data between the guest physical address (GPA) and the host virtual address (HVA).
 *
 * This function is used to copy data between the host memory and the guest physical
 * address (GPA) within the same physical machine. It is typically used when there is
 * a need to transfer data between the host and the guest locally. The function
 * internally calls the local_gpa2hpa and hpa2hva functions, which perform the actual
 * copying of data.
 *
 * It takes several parameters: a pointer to the VM structure (struct acrn_vm), a
 * pointer to the destination buffer (h_ptr), the guest physical address (GPA) to copy
 * from/to (gpa), the size of the data to copy (size), the fixed page size (fix_pg_size),
 * and a flag indicating whether to copy from the VM (cp_from_vm).
 *
 * The first step in the function is to convert the GPA to a host physical address
 * (HPA) using the local_gpa2hpa function. If the HPA is found to be invalid
 * (represented by the INVALID_HPA constant), an error message is printed, and the
 * function returns with a length of 0.
 *
 * If the HPA is valid, the function proceeds to determine the page size based on the
 * fix_pg_size parameter or the default page size obtained from local_gpa2hpa. It then
 * calculates the offset within the page and the length of the data to copy. The offset
 * is obtained by performing a bitwise AND operation between the GPA and the page size
 * minus 1. The length is determined by taking the minimum value between the remaining
 * size and the remaining space within the page.
 *
 * Next, the function converts the HPA to a host virtual address (HVA) using the hpa2hva
 * function. This allows direct access to the physical memory. Before performing the memory
 * copy, the stac function is called to enable the store operations to be atomic. Then,
 * depending on the cp_from_vm flag, either memcpy_s is used to copy data from the guest
 * memory to the destination buffer or vice versa. Finally, the clac function is called
 * to disable atomic store operations.
 *
 * The function returns the length of the copied data, which can be used for error checking
 * or further processing.
 *
 * @param[in] vm          The pointer to the virtual machine structure.
 * @param[in] h_ptr       The pointer to the host memory buffer.
 * @param[in] gpa         The guest physical address to copy the data to/from.
 * @param[in] size        The size of the data to be copied.
 * @param[in] fix_pg_size The fixed page size to be used for the copy.
 * @param[in] cp_from_vm  Boolean value indicating the direction of the copy. If true, the data is
 * copied from the VM to the host. If false, the data is copied from the host to the VM.
 *
 * @return Returns the length of the data that was copied.
 *
 * @pre vm != NULL
 * @pre h_ptr != NULL
 */
static inline uint32_t local_copy_gpa(struct acrn_vm *vm, void *h_ptr, uint64_t gpa,
	uint32_t size, uint32_t fix_pg_size, bool cp_from_vm)
{
	uint64_t hpa;
	uint32_t offset_in_pg, len, pg_size;
	void *g_ptr;

	hpa = local_gpa2hpa(vm, gpa, &pg_size);
	if (hpa == INVALID_HPA) {
		pr_err("%s,vm[%hu] gpa 0x%lx,GPA is unmapping",
			__func__, vm->vm_id, gpa);
		len = 0U;
	} else {

		if (fix_pg_size != 0U) {
			pg_size = fix_pg_size;
		}

		offset_in_pg = (uint32_t)gpa & (pg_size - 1U);
		len = (size > (pg_size - offset_in_pg)) ? (pg_size - offset_in_pg) : size;

		g_ptr = hpa2hva(hpa);

		stac();
		if (cp_from_vm) {
			(void)memcpy_s(h_ptr, len, g_ptr, len);
		} else {
			(void)memcpy_s(g_ptr, len, h_ptr, len);
		}
		clac();
	}

	return len;
}

/**
 * @brief Copies data between host memory and guest physical address (GPA).
 *
 * This function is used to copy data between the host memory and the guest
 * physical address (GPA). It is typically used when there is a need to transfer
 * data between the host and the guest. The function internally calls the
 * local_copy_gpa function, which performs the actual copying of data.
 *
 * The function takes several arguments: a pointer to the host memory (h_ptr_arg),
 * the guest physical address (gpa_arg), the size of the data to be copied
 * (size_arg), and a flag indicating whether to copy from the guest VM (cp_from_vm).
 *
 * The function starts by initializing local variables, including a length variable
 * (len) to keep track of the amount of data copied in each iteration. It also
 * initializes an error variable (err) to 0, indicating no error initially.
 *
 * The function then enters a while loop, which continues until the entire data size
 * (size) is copied. Inside the loop, it calls another function local_copy_gpa to copy
 * a portion of data between the guest physical address (gpa) and the host memory (h_ptr).
 * The len variable is updated with the actual amount of data copied in each iteration.
 *
 * If the len is 0, it means that the local_copy_gpa function failed to copy any data,
 * and the err variable is set to -EINVAL to indicate an invalid argument error. The
 * loop is then exited, and the function returns the error value.
 *
 * If the len is non-zero, the gpa, h_ptr, and size variables are updated accordingly
 * to reflect the progress of the data copying. The loop continues until the entire
 * data size is copied.
 *
 * Finally, the function returns the error value, which will be 0 if the data copying
 * was successful or a negative value if an error occurred.
 *
 * @param[in] vm         The pointer to the virtual machine structure.
 * @param[in] h_ptr_arg  The pointer to the host memory buffer.
 * @param[in] gpa_arg    The guest physical address to copy the data to/from.
 * @param[in] size_arg   The size of the data to be copied.
 * @param[in] cp_from_vm Boolean value indicating the direction of the copy. If true, the data is
 * copied from the VM to the host. If false, the data is copied from the host to the VM.
 *
 * @return Returns 0 on success, or -EINVAL if the local_copy_gpa function returns 0.
 *
 * @retval -EINVAL If the local_copy_gpa function returns 0.
 * @retval 0       If the copy operation is successful.
 *
 * @pre vm != NULL
 * @pre h_ptr_arg != NULL
 */
static inline int32_t copy_gpa(struct acrn_vm *vm, void *h_ptr_arg, uint64_t gpa_arg,
	uint32_t size_arg, bool cp_from_vm)
{
	void *h_ptr = h_ptr_arg;
	uint32_t len;
	uint64_t gpa = gpa_arg;
	uint32_t size = size_arg;
	int32_t err = 0;

	while (size > 0U) {
		len = local_copy_gpa(vm, h_ptr, gpa, size, 0U, cp_from_vm);
		if (len == 0U) {
			err = -EINVAL;
			break;
		}
		gpa += len;
		h_ptr += len;
		size -= len;
	}

	return err;
}

/**
 * @brief Copies data between guest virtual address (GVA) and host memory.
 *
 * This function copies data between the guest virtual address (GVA) and the host memory.
 * It iterates over the given size of data and copies it in chunks using the local_copy_gpa
 * function.
 *
 * The copy_gva function takes several arguments: a pointer to a struct acrn_vcpu
 * object named vcpu, a void pointer h_ptr_arg, an unsigned 64-bit integer gva_arg,
 * an unsigned 32-bit integer size_arg, a pointer to a 32-bit integer err_code, a
 * pointer to a 64-bit integer fault_addr, and a boolean value cp_from_vm. The cp_from_vm
 * indicates the diretion to copy data.
 *
 * Inside the function, some local variables such as h_ptr, gpa, ret, len, gva, and
 * size are declared. These variables are used to store intermediate values and track
 * the progress of the memory copying process.
 *
 * The function then enters a while loop that continues as long as the size argument
 * is greater than 0 and the ret variable is equal to 0. This loop is responsible for
 * copying memory between the guest virtual address (GVA) and the host pointer (h_ptr).
 *
 * Within the loop, the function calls the gva2gpa function, passing the vcpu, gva, gpa,
 * and err_code as arguments. This function is responsible for translating the guest
 * virtual address to a guest physical address (GPA). If the translation is successful
 * (i.e., ret is greater than or equal to 0), the function then calls local_copy_gpa to
 * copy the memory between the guest physical address and the host pointer. The len variable
 * stores the number of bytes copied.
 *
 * If len is not equal to 0, the function updates the gva, h_ptr, and size variables to
 * reflect the progress of the copying process. If len is 0, it means that the memory
 * copy was unsuccessful, and the function sets ret to -EINVAL (a negative value indicating
 * an invalid argument).
 *
 * If the translation from GVA to GPA fails, the function sets the fault_addr variable to
 * the current gva value and prints an error message using the pr_err function.
 *
 * Finally, the function returns the value of ret, which indicates the success or failure
 * of the memory copying process.
 *
 * @param[in]     vcpu       The pointer to the virtual CPU structure.
 * @param[in]     h_ptr_arg  The pointer to the host memory buffer.
 * @param[in]     gva_arg    The guest virtual address to copy the data to/from.
 * @param[in]     size_arg   The size of the data to be copied.
 * @param[in,out] err_code   Pointer to store the error code if an error occurs during the copy.
 * @param[out]    fault_addr Pointer to store the fault address if a fault occurs during the copy.
 * @param[in]     cp_from_vm Boolean value indicating the direction of the copy. If true, the data is
 * copied from the VM to the host. If false, the data is copied from the host to the VM.
 *
 * @return Returns 0 on success, or a negative error code on failure.
 *
 * @retval -EFAULT If there is fault during the copy operation.
 * @retval -EINVAL If the provided parameters are invalid or if the local_copy_gpa function returns 0.
 * @retval 0       If the copy operation is successful.
 *
 * @pre vcpu != NULL
 * @pre h_ptr_arg != NULL
 * @pre err_code != NULL
 * @pre fault_addr != NULL
 */
static inline int32_t copy_gva(struct acrn_vcpu *vcpu, void *h_ptr_arg, uint64_t gva_arg,
	uint32_t size_arg, uint32_t *err_code, uint64_t *fault_addr,
	bool cp_from_vm)
{
	void *h_ptr = h_ptr_arg;
	uint64_t gpa = 0UL;
	int32_t ret = 0;
	uint32_t len;
	uint64_t gva = gva_arg;
	uint32_t size = size_arg;

	while ((size > 0U) && (ret == 0)) {
		ret = gva2gpa(vcpu, gva, &gpa, err_code);
		if (ret >= 0) {
			len = local_copy_gpa(vcpu->vm, h_ptr, gpa, size, PAGE_SIZE_4K, cp_from_vm);
			if (len != 0U) {
				gva += len;
				h_ptr += len;
				size -= len;
			} else {
				ret =  -EINVAL;
				/* TODO: yisun1 - Shall break the loop and set fault_addr like below? */
			}
		} else {
			*fault_addr = gva;
			pr_err("error[%d] in GVA2GPA, err_code=0x%x", ret, *err_code);
			/* TODO: yisun1 - Shall break the loop? */
		}
	}

	return ret;
}

/**
 * @brief Copies data from the guest physical address (GPA) space of a virtual machine (VM)
 * to the host virutal address (HVA) space.
 *
 * The purpose of this function is to copy data from the guest physical address
 * (GPA) to a host virtual address (HVA) within a virtual machine (VM).
 *
 * The function takes in several parameters: a pointer to a structure acrn_vm, a
 * void pointer h_ptr, an unsigned 64-bit integer gpa, and an unsigned 32-bit integer
 * size. The function returns an int32_t value.
 *
 * Inside the function, there is a variable ret initialized to 0, which will be
 * used to store the return value of the copy_gpa function. The copy_gpa function
 * is called with the provided parameters vm, h_ptr, gpa, size, and an additional
 * argument of 1. The additional argument indicates that the copy operation is from
 * gpa.
 *
 * After calling copy_gpa, the return value ret is checked for equality with 0 using
 * the != operator. If the return value is not equal to 0, an error message is printed
 * using the pr_err function. The error message includes the values of gpa, vm->vm_id,
 * and (uint64_t)h_ptr.
 *
 * Finally, the function returns the value of ret, which could be either 0 indicating
 * a successful copy or a non-zero value indicating an error occurred during the copy
 * process.
 *
 * @param[in] vm    The pointer to the virtual machine structure.
 * @param[in] h_ptr The pointer to the destination buffer in the host physical address space.
 * @param[in] gpa   The guest physical address to copy from.
 * @param[in] size  The number of bytes to copy.
 *
 * @return 0 on success, or an error code on failure.
 *
 * @retval -EINVAL If the gpa is invalid.
 * @retval 0       If the copy operation is successful.
 *
 * @pre Caller(Guest) should make sure gpa is continuous.
 * - gpa from hypercall input which from kernel stack is gpa continuous, not
 *   support kernel stack from vmap
 * - some other gpa from hypercall parameters, VHM should make sure it's
 *   continuous
 * @pre vm != NULL
 * @pre h_ptr != NULL
 */
int32_t copy_from_gpa(struct acrn_vm *vm, void *h_ptr, uint64_t gpa, uint32_t size)
{
	int32_t ret = 0;

	ret = copy_gpa(vm, h_ptr, gpa, size, 1);
	if (ret != 0) {
		pr_err("Unable to copy GPA 0x%llx from VM%d to HPA 0x%llx\n", gpa, vm->vm_id, (uint64_t)h_ptr);
	}

	return ret;
}

/**
 * @brief Copies data from host virtual address (HVA) to guest physical address (GPA) in a virtual machine.
 *
 * The purpose of this function is to copy data from the host virtual address
 * (HVA) pointed to by h_ptr to the guest physical address (GPA) specified by
 * gpa in a virtual machine (acrn_vm). It uses another function called copy_gpa
 * to perform the actual copying.
 *
 * The function takes four parameters: a pointer to a structure of type acrn_vm,
 * a void pointer h_ptr, an unsigned 64-bit integer gpa, and an unsigned 32-bit
 * integer size. The function returns a signed 32-bit integer.
 *
 * The copy_gpa function is called with the vm parameter (which is a pointer to the
 * acrn_vm structure), h_ptr, gpa, size, and an additional argument of 0. The return
 * value of copy_gpa is stored in the ret variable.
 *
 * After calling copy_gpa, the code checks if the return value ret is not equal to 0.
 * If it is not equal to 0, an error message is printed using the pr_err function.
 * The error message includes the values of h_ptr, gpa, and the vm_id field of the
 * acrn_vm structure.
 *
 * Finally, the function returns the value of ret, which indicates the success or
 * failure of the copying operation.
 *
 * @param[in] vm    The pointer to the virtual machine structure.
 * @param[in] h_ptr The pointer to the source data in host physical address space.
 * @param[in] gpa   The guest physical address to copy the data to.
 * @param[in] size  The size of the data to be copied, in bytes.
 *
 * @return 0 on success, or an error code on failure.
 *
 * @retval -EINVAL If the gpa is invalid.
 * @retval 0       If the copy operation is successful.
 *
 * @pre Caller(Guest) should make sure gpa is continuous.
 * - gpa from hypercall input which from kernel stack is gpa continuous, not
 *   support kernel stack from vmap
 * - some other gpa from hypercall parameters, VHM should make sure it's
 *   continuous
 * @pre vm != NULL
 * @pre h_ptr != NULL
 */
int32_t copy_to_gpa(struct acrn_vm *vm, void *h_ptr, uint64_t gpa, uint32_t size)
{
	int32_t ret = 0;

	ret = copy_gpa(vm, h_ptr, gpa, size, 0);
	if (ret != 0) {
		pr_err("Unable to copy HPA 0x%llx to GPA 0x%llx in VM%d\n", (uint64_t)h_ptr, gpa, vm->vm_id);
	}

	return ret;
}

/**
 * @brief Copies data from guest virtual address (GVA) to host memory.
 *
 * This function copies data from the guest virtual address (GVA) to the host memory.
 * It is used in the ACRN hypervisor to handle memory operations for a virtual CPU.
 *
 * The function takes several parameters: a pointer to a struct acrn_vcpu object
 * named vcpu, a void pointer h_ptr, an unsigned 64-bit integer gva, two unsigned
 * 32-bit integers size and err_code, and a pointer to an unsigned 64-bit integer
 * fault_addr. The function returns a signed 32-bit integer.
 *
 * The function internally calls another function named copy_gva with the same
 * parameters, except for an additional boolean flag set to 1. This flag is used
 * to indicate that the copy operation is from the guest virtual address to the
 * host address.
 *
 * @param[in]     vcpu       The pointer to the virtual CPU structure.
 * @param[in]     h_ptr      The pointer to the host memory where the data will be copied to.
 * @param[in]     gva        The guest virtual address from where the data will be copied.
 * @param[in]     size       The size of the data to be copied.
 * @param[in,out] err_code   A pointer to store the error code if an error occurs during the copy operation.
 * @param[out]    fault_addr A pointer to store the fault address if an error occurs during the copy operation.
 *
 * @return The number of bytes copied on success, or a negative error code on failure.
 *
 * @retval -EFAULT If there is fault during the copy operation.
 * @retval -EINVAL If the provided parameters are invalid.
 * @retval 0       If the copy operation is successful.
 *
 * @pre vcpu != NULL
 * @pre h_ptr != NULL
 * @pre err_code != NULL
 * @pre fault_addr != NULL
 */
int32_t copy_from_gva(struct acrn_vcpu *vcpu, void *h_ptr, uint64_t gva,
	uint32_t size, uint32_t *err_code, uint64_t *fault_addr)
{
	return copy_gva(vcpu, h_ptr, gva, size, err_code, fault_addr, 1);
}

/**
 * @brief Copies data from the host memory to the guest virtual address (GVA).
 *
 * The purpose of this function is to copy data from the host memory to
 * the guest virtual address (GVA).
 *
 * The function takes several parameters: a pointer to a struct acrn_vcpu
 * object named vcpu, a void pointer h_ptr, an unsigned 64-bit integer gva,
 * two unsigned 32-bit integers size and err_code, and a pointer to an
 * unsigned 64-bit integer fault_addr. The function returns a signed 32-bit
 * integer.
 *
 * Inside the function, there is a call to another function named copy_gva.
 * This function is being invoked with the provided parameters, along with
 * an additional boolean argument set to false. copy_gva is responsible for
 * performing the actual copying of data from the host memory to the guest
 * virtual address.
 *
 * The return value of copy_gva is being directly returned by the copy_to_gva
 * function.
 *
 * @param[in]     vcpu       The pointer to the virtual CPU structure.
 * @param[in]     h_ptr      The pointer to the host memory buffer.
 * @param[in]     gva        The guest virtual address to copy the data to.
 * @param[in]     size       The size of the data to be copied.
 * @param[in,out] err_code   Pointer to store the error code if an error occurs during the copy.
 * @param[out]    fault_addr Pointer to store the fault address if a fault occurs during the copy.
 *
 * @return Returns 0 on success, or a negative error code on failure.
 *
 * @retval -EFAULT If there is fault during the copy operation.
 * @retval -EINVAL If the provided parameters are invalid.
 * @retval 0       If the copy operation is successful.
 *
 * @pre vcpu != NULL
 * @pre h_ptr != NULL
 * @pre err_code != NULL
 * @pre fault_addr != NULL
 */
int32_t copy_to_gva(struct acrn_vcpu *vcpu, void *h_ptr, uint64_t gva,
	uint32_t size, uint32_t *err_code, uint64_t *fault_addr)
{
	return copy_gva(vcpu, h_ptr, gva, size, err_code, fault_addr, false);
}

/**
 * @brief Convert a guest physical address (GPA) to a host virtual address (HVA).
 *
 * This function takes a virtual machine (VM) and a guest physical address (GPA)
 * as input, and returns the corresponding host virtual address (HVA) if the
 * GPA is valid. If the GPA is invalid, it returns NULL.
 *
 * The function takes two parameters: a pointer to a structure of type acrn_vm
 * and an unsigned 64-bit integer x.
 *
 * The function calls function gpa2hpa which takes the vm pointer and x as
 * arguments to convert the guest physical address (GPA) to a host physical address
 * (HPA). The result of this conversion is stored in the local variable hpa.
 *
 * Next, there is a conditional statement that checks if the value of hpa is
 * equal to a constant value INVALID_HPA. If it is, the function returns NULL,
 * indicating that the conversion was unsuccessful. Otherwise, it calls another
 * function hpa2hva with hpa as an argument. This function is responsible for
 * converting the host physical address (HPA) to a host virtual address (HVA).
 *
 * If the conversion was successful, the function will return the host
 * virtual address (HVA) corresponding to the given guest physical address (GPA).
 * Otherwise, it will return NULL.
 *
 * @param[in] vm The virtual machine for which the conversion is performed.
 * @param[in] x  The guest physical address (GPA) to be converted.
 *
 * @return The host virtual address (HVA) corresponding to the GPA, or NULL if the GPA is invalid.
 *
 * @pre vm != NULL
 */
void *gpa2hva(struct acrn_vm *vm, uint64_t x)
{
	uint64_t hpa = gpa2hpa(vm, x);
	return (hpa == INVALID_HPA) ? NULL : hpa2hva(hpa);
}
