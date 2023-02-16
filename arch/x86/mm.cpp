/* SPDX-License-Identifier: MIT */

/*
 * arch/x86/mm.cpp
 *
 * InfOS
 * Copyright (C) University of Edinburgh 2016.  All Rights Reserved.
 *
 * Tom Spink <tspink@inf.ed.ac.uk>
 */
#include <arch/x86/init.h>
#include <arch/x86/multiboot.h>
#include <arch/x86/cpuid.h>
#include <arch/x86/irq.h>
#include <arch/x86/context.h>
#include <arch/x86/vma.h>
#include <infos/kernel/kernel.h>
#include <infos/kernel/thread.h>
#include <infos/kernel/process.h>
#include <infos/kernel/log.h>
#include <infos/mm/mm.h>
#include <infos/mm/page-allocator.h>
#include <infos/util/string.h>
#include <arch/arch.h>
#include <arch/x86/x86-arch.h>

using namespace infos::arch::x86;
using namespace infos::kernel;
using namespace infos::mm;
using namespace infos::util;

extern "C" infos::kernel::Thread *current_thread;

/**
 * Page fault handler
 * @param irq The IRQ object associated with this exception.
 * @param priv Private data associated with this exception.
 */
static void handle_page_fault(const IRQ *irq, void *priv)
{
	// Retrieve the fault_address from the cr2 control register.
	uint64_t fault_address;
	asm volatile("mov %%cr2, %0" : "=r"(fault_address));

	if (current_thread == NULL) {
		// If there is no current_thread, then this page fault happened REALLY
		// early.  We must abort.
		syslog.messagef(LogLevel::FATAL, "*** PAGE FAULT @ vaddr=%p", fault_address);

		arch_abort();
	}

	// If there is a current thread, abort it.
	syslog.messagef(LogLevel::WARNING, "*** PAGE FAULT @ vaddr=%p rip=%p proc=%s", fault_address, current_thread->context().native_context->rip, current_thread->owner().name().c_str());

	// TODO: support passing page-faults into threads.
	current_thread->owner().terminate(-1);
}

// Ugly hack
uint64_t *infos::arch::x86::__template_pml4;

/* In define.h (wrong place!) we have the following.
#define KERNEL_VMEM_START	((uintptr_t)0xFFFFFFFF80000000u)
#define KERNEL_VMEM_END		((uintptr_t)0xFFFFFFFFFFFFFFFFu)
 *
 * The kernel is linked in the range starting at 0xFFFFFFFF80000000
 * (binary: 33 ones followed by 31 zeroes),
 * and we create the kernel mapping in every address space.
 * Since this mapping is created without the 'USER' flag, user-mode
 * code cannot access it. See arch/x86/mm.c.
 *
 * On 64-bit x86, virtual addresses are really 48-bit numbers padded
 * out like this.
 *
   63|62         47|46                                           0|
   .--------------------------------------------------------------.
   | |xxxxxxxxxxxxx|                                              |
   '--------------------------------------------------------------'
   ^  ^.            ^-----------------lower bits------------------^
   |    'non-canonical bits'
   top bit

 * The non-canonical bits are ignored in address translation, except
 * that they must match the top bit, i.e. be all zeroes or all ones.
 *
 * It follows that our FFFFFFFF8 prefix, of 33 ones, gets broken down as
 * one top bit, then 16 non-canonical bits that are *ignored*, then
 * 16 ones. From the page tables' point of view, it is a sequence of
 * only 17 ones.
 
 * Since each layer of page table has 512 (2^9) entries, our 17-bit
 * prefix refers to
 * PML4 entry MAX  (first 9 bits),
 * PDP entries MAX-1 (next 9 bits, which are all 1 except for the last one),
 *        and  MAX   (because we go all the way up to FFFFFFFFFFFFFFFF)
 *
 * In hex, MAX and MAX-1 are respectively 0x1ff and 0x1fe.
 *
 * Also in define.h we have
#define PMEM_VA_START		((uintptr_t)0xFFFF800000000000)
#define PMEM_VA_END			((uintptr_t)0xFFFF800100000000)
#define PMEM_VA_SIZE		(PMEM_VA_END - PMEM_VA_START)
 *
 * meaning that in every process we also create a contiguous mapping of
 * 4GB of physical memory. Using the same reasoning as above, we conclude
 * that the prefixes FFFF8000 and FFFF8001, which lead with 17 ones (but
 * only a single one that is canonical), refer to
 * PML4 entry 0x100 == 0b1 ... 000 0000 0 (first 9 bits: top bit, then bits below the 8),
 * and PDP entries     0b  000 0000 00 <-- PMEM_VA_START's next 9 bits
 *                     0b  000 0000 01
 *                     0b  000 0000 10
 *                     0b  000 0000 11
 * up to but not incl  0b  000 0001 00 <-- PMEM_VA_END
 * i.e. PDP entries 0 to 3 inclusive.
 *
 * Each PDP entry maps 1GB of address space, i.e. 2^(2*9 + 12) bytes.
 */

/**
 * Usurps the initial page tables, and introduces new ones for the
 * higher mapping.
 * @return Returns true if the page tables were successfully reinitialised,
 * or false otherwise.
 */
static bool reinitialise_pgt()
{
	// Allocate sixteen frames to mess around with.
	const FrameDescriptor *frames = sys.mm().pgalloc().allocate(4);
	if (!frames) {
		x86_log.message(LogLevel::ERROR, "The page allocator has not worked!");
		return false;
	}

	// The PML4 will be the first page.
	uint64_t *pml4 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[0]);

	// Zero all the frames.
	bzero(pml4, (1 << 4) * 0x1000);

	x86_log.messagef(LogLevel::DEBUG, "Kernel page tables @ %p", pml4);

	// Populate some local variables that will make working with the various
	// page tables at the various levels a bit easier.
	uint64_t *pdp0 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[1]);
	uint64_t *pdp1 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[2]);

	uint64_t *pdp0_pd0 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[3]);
	uint64_t *pdp0_pd1 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[4]);

	uint64_t *pdp1_pd0 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[5]);
	uint64_t *pdp1_pd1 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[6]);
	uint64_t *pdp1_pd2 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[7]);
	uint64_t *pdp1_pd3 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_kva(&frames[8]);

	/* Summary of the entries we need to create (see comment above).
	 *
	 * PML4[0x1ff] -> pdp0       the kernel VMA
	 * PML4[0x100] -> pdp1       the physical 4GB mapping
	 *
	 * pdp0[0x0] -> pd0          kernel VMA low part   \ two PDs, each 2^18 pages a.k.a. 1GB...
	 * pdp0[0x1] -> pd1          kernel VMA next part  / ... so we've mapped 2GB here
	 *
	 * pdp1[0x0] -> pd0          physical address space low  1GB \ share the PDs of kernel mapping!
	 * pdp1[0x1] -> pd1          physical address space next 1GB /
	 * pdp1[0x2] -> pd2          physical address space next 1GB -- fresh PD
	 * pdp1[0x3] -> pd3          physical address space next 1GB -- fresh PD
	 */

	// Fill in the PML4 and PDPs.
	pml4[0x1ff] = kva_to_pa((virt_addr_t)pdp0) | PTE_PRESENT | PTE_WRITABLE;
	pml4[0x100] = kva_to_pa((virt_addr_t)pdp1) | PTE_PRESENT | PTE_WRITABLE;

	pdp0[0x1fe] = kva_to_pa((virt_addr_t)pdp0_pd0) | PTE_PRESENT | PTE_WRITABLE;
	pdp0[0x1ff] = kva_to_pa((virt_addr_t)pdp0_pd1) | PTE_PRESENT | PTE_WRITABLE;

	pdp1[0x000] = kva_to_pa((virt_addr_t)pdp1_pd0) | PTE_PRESENT | PTE_WRITABLE;
	pdp1[0x001] = kva_to_pa((virt_addr_t)pdp1_pd1) | PTE_PRESENT | PTE_WRITABLE;
	pdp1[0x002] = kva_to_pa((virt_addr_t)pdp1_pd2) | PTE_PRESENT | PTE_WRITABLE;
	pdp1[0x003] = kva_to_pa((virt_addr_t)pdp1_pd3) | PTE_PRESENT | PTE_WRITABLE;

	// Now, fill in the KERNEL PDs. Each has 512 entries
	// and we fill in both page directores in lockstep.
	// They are 1GB i.e. 0x40000000 i.e. 2^30 bytes apart, and
	// each PD entry covers 2^(9+12) i.e. 0x200000 bytes (2MB).
	// We use a HUGE (2GB) mapping to save creating bottom-level PTs.
	uintptr_t addr = 0;
	for (unsigned int i = 0; i < 1<<9; i++) {
		pdp0_pd0[i] = addr | PTE_PRESENT | PTE_WRITABLE | PTE_HUGE;
		pdp0_pd1[i] = (addr + (1<<30)) | PTE_PRESENT | PTE_WRITABLE | PTE_HUGE;;
		addr += 1<<21;
	}

	// Now, fill in the PHYSMEM PDs for a 4G mapping, similarly.
	addr = 0;
	for (unsigned int i = 0; i < 1<<9; i++) {
		pdp1_pd0[i] = addr | PTE_PRESENT | PTE_WRITABLE | PTE_HUGE;
		addr += 1<<21;
	}

	for (unsigned int i = 0; i < 1<<9; i++) {
		pdp1_pd1[i] = addr | PTE_PRESENT | PTE_WRITABLE | PTE_HUGE;
		addr += 1<<21;
	}

	for (unsigned int i = 0; i < 1<<9; i++) {
		pdp1_pd2[i] = addr | PTE_PRESENT | PTE_WRITABLE | PTE_HUGE;
		addr += 1<<21;
	}

	for (unsigned int i = 0; i < 1<<9; i++) {
		pdp1_pd3[i] = addr | PTE_PRESENT | PTE_WRITABLE | PTE_HUGE;
		addr += 1<<21;
	}

	// Welp, here we go.  Reload the page tables
	asm volatile ("mov %0, %%cr3" :: "r"(kva_to_pa((virt_addr_t)pml4)));

	// Grab a pointer to the PML4, to use as a template when creating process page tables
	__template_pml4 = (uint64_t *)sys.mm().pgalloc().pfdescr_to_vpa(&frames[0]);

	return true;
}

/**
 * Initialises the memory management subsystem.
 * @return Returns true if initialisation was successful, false otherwise.
 */
bool infos::arch::x86::mm_init()
{
	for (unsigned int i = 0; i < multiboot_info_structure->mmap_length / sizeof(struct multiboot_mmap_entry); i++) {
		struct multiboot_mmap_entry *entry = (struct multiboot_mmap_entry *)pa_to_kva(multiboot_info_structure->mmap_addr + (sizeof(struct multiboot_mmap_entry) * i));

		if ((entry->len & ~0xfff) != 0) {
			MemoryType::MemoryType type = MemoryType::NORMAL;

			switch (entry->type) {
			case MULTIBOOT_MEMORY_AVAILABLE:
				type = MemoryType::NORMAL;
				break;
			case MULTIBOOT_MEMORY_RESERVED:
				type = MemoryType::UNUSABLE;
				break;
			default:
				x86_log.messagef(LogLevel::WARNING, "Skipping memory block with unknown type");
				continue;
			}

			sys.mm().add_physical_memory(entry->addr, (entry->len >> 12), type);
		}
	}

	if (!sys.mm().initialise_allocators())
		return false;

	return reinitialise_pgt();
}

bool infos::arch::x86::mm_pf_init()
{
	// Register the page-fault handler
	return x86arch.irq_manager().install_exception_handler(IRQ_PAGE_FAULT, handle_page_fault, NULL);
}

/* Pasted from vma.cpp, which used to be x86-specific. At least now the
 * arch-dep code is in a sane place in the tree. It is nasty that these
 * definitions are still namespaced to the global VMA, not anything
 * x86-specific. */
void VMA::install_default_kernel_mapping()
{
	assert(__template_pml4);
	
	uint64_t *pgt = (uint64_t *)_pgt_virt_base;
	
	pgt[0x1ff] = __template_pml4[0x1ff];
	pgt[0x100] = __template_pml4[0x100];
}

void infos::mm::VMA::insert_mapping(virt_addr_t va, phys_addr_t pa, MappingFlags::MappingFlags flags)
{
	table_idx_t pml4_idx, pdp_idx, pd_idx, pt_idx;
	va_table_indices(va, pml4_idx, pdp_idx, pd_idx, pt_idx);
	
	PML4TableEntry *pml4 = &((PML4TableEntry *)_pgt_virt_base)[pml4_idx];
	
	if (pml4->base_address() == 0) {
		auto pdp = allocate_phys(0);
		assert(pdp);
		
		pml4->base_address(sys.mm().pgalloc().pfdescr_to_pa(pdp));
		pml4->present(true);
		pml4->writable(true);
		pml4->user(true);
	}
	
	PDPTableEntry *pdp = &((PDPTableEntry *)pa_to_vpa(pml4->base_address()))[pdp_idx];
	
	if (pdp->base_address() == 0) {
		auto pd = allocate_phys(0);
		assert(pd);
		
		pdp->base_address(sys.mm().pgalloc().pfdescr_to_pa(pd));
		pdp->present(true);
		pdp->writable(true);
		pdp->user(true);
	}
	
	PDTableEntry *pd = &((PDTableEntry *)pa_to_vpa(pdp->base_address()))[pd_idx];
	
	if (pd->base_address() == 0) {
		auto pt = allocate_phys(0);
		assert(pt);
		
		pd->base_address(sys.mm().pgalloc().pfdescr_to_pa(pt));
		pd->present(true);
		pd->writable(true);
		pd->user(true);
	}
	
	PTTableEntry *pt = &((PTTableEntry *)pa_to_vpa(pd->base_address()))[pt_idx];
	
	pt->base_address(pa);
	
	if (flags & MappingFlags::Present) pt->present(true);
	if (flags & MappingFlags::Writable) pt->writable(true);
	if (flags & MappingFlags::User) pt->user(true);
	
	mm_log.messagef(LogLevel::DEBUG, "vma: mapping va=%p -> pa=%p", va, pa);
}

FrameDescriptor *infos::mm::VMA::allocate_phys(int order)
{
	auto pfdescr = sys.mm().pgalloc().allocate(order);
	if (!pfdescr) return NULL;
	
	FrameAllocation pa;
	pa.descriptor_base = pfdescr;
	pa.allocation_order = order;
	
	_frame_allocations.append(pa);
	pnzero((void *)sys.mm().pgalloc().pfdescr_to_vpa(pa.descriptor_base), 1 << order);
	
	return pfdescr;
}

bool infos::mm::VMA::allocate_virt_any(int nr_pages, int flags /* = -1 */)
{
	return false;
}

bool infos::mm::VMA::allocate_virt(virt_addr_t va, int nr_pages, int flags /* = -1 */)
{
	if (nr_pages == 0) return false;
	
	int order = __log2ceil(nr_pages);
	
	const FrameDescriptor *pfdescr = allocate_phys(order);
	if (!pfdescr) {
		return false;
	}
	
	virt_addr_t vbase = va;
	phys_addr_t pbase = sys.mm().pgalloc().pfdescr_to_pa(pfdescr);
	for (unsigned int i = 0; i < (1u << order); i++) {
		insert_mapping(vbase, pbase, MappingFlags::Present | MappingFlags::User | MappingFlags::Writable);
		
		vbase += 0x1000;
		pbase += 0x1000;
	}
	
	return true;
}

bool infos::mm::VMA::is_mapped(virt_addr_t va)
{
	phys_addr_t pa;
	return get_mapping(va, pa);
}

bool infos::mm::VMA::get_mapping(virt_addr_t va, phys_addr_t& pa)
{
	if (!_pgt_virt_base) return false;
	
	table_idx_t pml4_idx, pdp_idx, pd_idx, pt_idx;
	va_table_indices(va, pml4_idx, pdp_idx, pd_idx, pt_idx);
	
	PML4TableEntry *pml4 = &((PML4TableEntry *)_pgt_virt_base)[pml4_idx];
	
	if (!pml4->present()) {
		return false;
	}
	
	PDPTableEntry *pdp = &((PDPTableEntry *)pa_to_vpa(pml4->base_address()))[pdp_idx];
	
	if (!pdp->present()) {
		return false;
	}
	
	PDTableEntry *pd = &((PDTableEntry *)pa_to_vpa(pdp->base_address()))[pd_idx];
	
	if (!pd->present()) {
		return false;
	}
	
	PTTableEntry *pt = &((PTTableEntry *)pa_to_vpa(pd->base_address()))[pt_idx];
	
	if (!pt->present()) {
		return false;
	}
	
	pa = pt->base_address() | __page_offset(va);
	return true;
}


bool infos::mm::VMA::copy_to(virt_addr_t dest_va, const void* src, size_t size)
{
	phys_addr_t pa;
	if (!get_mapping(dest_va, pa))
		return false;
	
	void *dest = (void *)pa_to_vpa(pa);
	
	//mm_log.messagef(LogLevel::DEBUG, "vma: copy-to dst=va=%p:pa=%p:vpa=%p src=%p size=%p", dest_va, pa, dest, src, size);
	memcpy(dest, src, size);
	
	return true;
}

void infos::mm::VMA::dump()
{
	PML4TableEntry *te = (PML4TableEntry *)_pgt_virt_base;
	for (unsigned int i = 0; i < 0x200; i++) {
		if (!te[i].present()) continue;
		if (te[i].huge()) {
			uintptr_t va = (uint64_t)i << 36;
			mm_log.messagef(LogLevel::DEBUG, "VMA: MAP VA=%p -> PA=%p", va, te[i].base_address());
		} else {
			dump_pdp(i, pa_to_vpa(te[i].base_address()));
		}
	}
}

void infos::mm::VMA::dump_pdp(int pml4, virt_addr_t pdp_va)
{
	PDPTableEntry *te = (PDPTableEntry *)pdp_va;
	for (unsigned int i = 0; i < 0x200; i++) {
		if (!te[i].present()) continue;
		if (te[i].huge()) {
			uintptr_t va = (uint64_t)pml4 << 36 | (uint64_t)i << 28;
			mm_log.messagef(LogLevel::DEBUG, "VMA: MAP VA=%p -> PA=%p", va, te[i].base_address());	
		} else {
			dump_pd(pml4, i, pa_to_vpa(te[i].base_address()));
		}
	}
}

void infos::mm::VMA::dump_pd(int pml4, int pdp, virt_addr_t pd_va)
{
	PDTableEntry *te = (PDTableEntry *)pd_va;
	for (unsigned int i = 0; i < 0x200; i++) {
		if (!te[i].present()) continue;
		if (te[i].huge()) {
			uintptr_t va = (uint64_t)pml4 << 36 | (uint64_t)pdp << 28 | (uint64_t)i << 20;
			mm_log.messagef(LogLevel::DEBUG, "VMA: MAP VA=%p -> PA=%p", va, te[i].base_address());
		} else {
			dump_pt(pml4, pdp, i, pa_to_vpa(te[i].base_address()));
		}
	}
}

void VMA::dump_pt(int pml4, int pdp, int pd, virt_addr_t pt_va)
{
	PTTableEntry *te = (PTTableEntry *)pt_va;
	for (unsigned int i = 0; i < 0x200; i++) {
		if (!te[i].present()) continue;
		
		uintptr_t va = (uint64_t)pml4 << 36 | (uint64_t)pdp << 28 | (uint64_t)pd << 20 | (uint64_t)i << 12;
		mm_log.messagef(LogLevel::DEBUG, "VMA: MAP VA=%p -> PA=%p", va, te[i].base_address());
	}
}
