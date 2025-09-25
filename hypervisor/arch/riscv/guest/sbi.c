/*
 * Copyright (C) 2023-2024 Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Authors:
 *   Haicheng Li <haicheng.li@intel.com>
 */

#include <lib/types.h>
#include <asm/lib/bits.h>
#include <asm/cpu.h>
#include <asm/guest/vcpu.h>
#include <asm/guest/vm.h>
#include "sbi.h"

/* TODO: this file is not complete. It is just for indicating how to handle IPI now.
 * For timer irq, Haoyu should take care of it because we support sstc and there is
 * no need to call SBI.
 */
static void send_vipi_mask(struct acrn_vcpu *vcpu, uint64_t mask, uint64_t base)
{
	uint16_t offset;

	offset = ffs64(mask);

	while ((offset + base) < vcpu->vm->hw.created_vcpus) {
		struct acrn_vcpu *t = &vcpu->vm->hw.vcpu[base + offset];

		clear_bit(offset, &mask);
		/* Send IPI to target vcpu */
		send_ipi(t);
		offset = ffs64(mask);
	}
}

static void sbi_ipi_handler(struct acrn_vcpu *vcpu, struct cpu_regs *regs)
{
	unsigned long *ret = &regs->a0;
	unsigned long funcid = regs->a6;

	if (funcid == SBI_TYPE_IPI_SEND_IPI) {
		send_vipi_mask(vcpu, regs->a0, regs->a1);
		*ret = SBI_SUCCESS;
	} else {
		*ret = SBI_ENOTSUPP;
	}

	return;
}
