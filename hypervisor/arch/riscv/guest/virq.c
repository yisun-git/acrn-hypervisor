/*
 * Copyright (C) 2023-2024 Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Authors:
 *   Haicheng Li <haicheng.li@intel.com>
 */

#include <types.h>
#include <errno.h>
#include <asm/lib/bits.h>
#include <asm/irq.h>
#include <asm/vmx.h>
#include <asm/guest/vcpu.h>
#include <asm/guest/vmcs.h>
#include <asm/guest/vm.h>
#include <asm/guest/virq.h>
#include <trace.h>
#include <logmsg.h>

#define CSR_SIE_SSIE		(1 << 1)
#define CSR_SIE_STIE		(1 << 5)
#define CSR_SIE_SEIE		(1 << 9)
#define CSR_SIE_MASK		(CSR_SIE_SSIE | CSR_SIE_STIE | CSR_SIE_SEIE)

#define CSR_SIP_SSIP		(1 << 1)

#define CSR_SCAUSE_IRQ		(1 << 63)

#define CSR_HSTATUS_SPV		(1 << 7)

#define CSR_VSSTATUS_SIE	(1 << 1)
#define CSR_VSSTATUS_SPIE	(1 << 5)
#define CSR_VSSTATUS_SPP	(1 << 8)

void send_ipi(struct acrn_vcpu *vcpu)
{
	struct guest_cpu_context *ctx = &vcpu->arch.contexts[vcpu->arch.cur_context];

	/* TODO: shall we add run_ctx.hvip? Then, we can directly handle hvip but not using sip. */
	ctx->run_ctx.sip |= CSR_SIP_SSIP;
	signal_event(&(vcpu->events[VCPU_EVENT_VIRTUAL_INTERRUPT]));
	/*
	 * TODO: It looks like we onlys support SBI IPI now, there is no need to call vcpu_make_request() like
	 * x86 todo. We can directly set sip above and the sip will be written into hvip in load_guest_state()
	 * before vmenter, RISC-V hvip setting can automatically generate interrupt for guest.
	 */
	//vcpu_make_request(vcpu, ACRN_REQUEST_EVENT);
}

static bool is_guest_irq_enabled(struct acrn_vcpu *vcpu)
{
	uint64_t ie = 0;
	struct run_context *ctx =
		&vcpu->arch.contexts[vcpu->arch.cur_context].run_ctx;

	ie = ctx->sie & CSR_SIE_MASK;
	pr_dbg("%s: ie 0x%lx", __func__, ie);

	return !!ie;
}

static bool vcpu_inject_exception(struct acrn_vcpu *vcpu)
{
	bool injected = false;
	uint64_t scause = cpu_csr_read(scause);
	/* TODO: shall we read hstatus in save_guest_state() by adding a new memeber, hstatus, for
	 * vcpu->arch.contexts[vcpu->arch.cur_context]? 
	 */
	uint64_t hstatus = cpu_csr_read(hstatus);
	uint64_t vsstatus = cpu_csr_read(vsstatus);
	struct guest_cpu_context *ctx = &vcpu->arch.contexts[vcpu->arch.cur_context];

	/*
	 * Handling exceptions which calls vcpu_make_request(vcpu, ACRN_REQUEST_EXCP):
	 * HX_EXIT_INS_MISALIGN
	 * HX_EXIT_INS_ACCESS
	 * HX_EXIT_INS_ILLEGAL
	 * HX_EXIT_BREAKPOINT
	 * HX_EXIT_LOAD_MISALIGN
	 * HX_EXIT_STORE_MISALIGN
	 * HX_EXIT_STORE_ACCESS
	 *
	 * SBI (HX_EXIT_ECALL_VS) is handled by SBI path.
	 * 
	 * MMIO accesses (HX_EXIT_PF_GUEST_INS/HX_EXIT_PF_GUEST_LOAD/HX_EXIT_PF_GUEST_STORE) are handled by mmio path.
	 *
	 * TODO: HX_EXIT_VIRT_INS is handled by hlt_vmexit_handler now. But it looks not right. According to Linux,
	 * we need handle the invalid instruction but not halt the vcpu.
	 *
	 * TODO: HX_EXIT_PF_INS/HX_EXIT_PF_LOAD/HX_EXIT_PF_STORE handling looks missed.
	 */
	/* Check if it is an exception but not interrupt */
	if (bitmap_test_and_clear_lock(ACRN_REQUEST_EXCP, &vcpu->arch.pending_req) &&
	    !(scause & CSR_SCAUSE_IRQ)) {
		/* SPV bit must be set if trap into hs-mode */
		if (hstatus & HSTATUS_SPV) {
			/* Change Guest SSTATUS.SPP bit */
			vsstatus &= ~CSR_VSSTATUS_SPP;
			if (ctx->run_ctx.sstatus & CSR_VSSTATUS_SPP) {
				vsstatus |= CSR_VSSTATUS_SPP;
			}

			/* Change Guest SSTATUS.SPIE bit */
			vsstatus &= ~CSR_VSSTATUS_SPIE;
			if (ctx->run_ctx.sstatus & CSR_VSSTATUS_SIE) {
				vsstatus |=CSR_VSSTATUS_SPIE;
			}

			/* Clear Guest SSTATUS.SIE bit */
			vsstatus &= ~CSR_VSSTATUS_SIE;

			/* Update Guest SSTATUS */
			cpu_csr_write(vsstatus, vsstatus);

			/* Update Guest SCAUSE, STVAL, and SEPC */
			cpu_csr_write(vscause, ctx->run_ctx.scause);
			cpu_csr_write(vstval, ctx->run_ctx.stval);
			cpu_csr_write(vsepc, ctx->run_ctx.sepc);

			/* Set Guest PC to Guest exception vector */
			ctx->run_ctx.sepc = cpu_csr_read(vstvec);

			/* Set Guest privilege mode to supervisor */
			ctx->run_ctx.sstatus |= CSR_VSSTATUS_SPP;

			injected = true;
		}
	}

	return injected;
}

static void acrn_inject_pending_intr(struct acrn_vcpu *vcpu, uint64_t *pending_req_bits, bool injected)
{
	if (is_guest_irq_enabled(vcpu) && (!injected)) {
		/* TODO: Bosheng shall implement external irq logics.
		 * Furthermore, shall we combine EXTINT and EVENT handling to one way. Linux handling all
		 * interrupts in same flow: save pending irqs into SW hvip, then compare HW hvip with SW
		 * hvip after VM Exit, set the different bits into HW hvip so that guest can get interrupts
		 * after VM Enter.
 		 */
		/* Inject external interrupt first */
		if (bitmap_test_and_clear_lock(ACRN_REQUEST_EXTINT, pending_req_bits)) {
			/* has pending external interrupts */
			vcpu_inject_extint(vcpu);
		}

		/* Handle IPI and timer irq */
		if (bitmap_test_and_clear_lock(ACRN_REQUEST_EVENT, pending_req_bits)) {
			/*
			 * IPI: send_ipi() has demonstrated how to handle IPI.
			 *
			 * Timer: As we support SSTC, there is no need to make request for timer interrupt
			 * either.
			 *
			 * So, we don't need handle anything here.
			 */
		}
	}
}

int32_t acrn_handle_pending_request(struct acrn_vcpu *vcpu)
{
	bool injected = false;
	int32_t ret = 0;
	struct acrn_vcpu_arch *arch = &vcpu->arch;
	uint64_t *pending_req_bits = &arch->pending_req;

	/* TODO:
	 * May change INIT_VMCS to a better name because riscv doesn't have vmcs concept.
	 * Yifan may implement init_vmcs related things.
	 */
	/* make sure ACRN_REQUEST_INIT_VMCS handler as the first one */
	if (bitmap_test_and_clear_lock(ACRN_REQUEST_INIT_VMCS, pending_req_bits)) {
		init_vmcs(vcpu);
	}

	if (bitmap_test_and_clear_lock(ACRN_REQUEST_TRP_FAULT, pending_req_bits)) {
		pr_fatal("Tiple fault happen -> shutdown!");
		ret = -EFAULT;
	} else {
		/*
		 * Inject pending exception prior pending interrupt to complete the previous instruction.
		 */
		injected = vcpu_inject_exception(vcpu);
		acrn_inject_pending_intr(vcpu, pending_req_bits, injected);
	}

	return ret;
}
