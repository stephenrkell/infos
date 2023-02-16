/* SPDX-License-Identifier: MIT */

/*
 * arch/x86/vma.h
 * 
 * InfOS
 * Copyright (C) University of Edinburgh 2016.  All Rights Reserved.
 * 
 * Tom Spink <tspink@inf.ed.ac.uk>
 *
 * This file cobbled together by Stephen Kell <srk31@srcf.ucam.org>
 */

#pragma once

namespace infos {
        namespace arch {
                namespace x86 {

extern uint64_t *__template_pml4;

#define BITS(val, start, end) ((((uint64_t)val) >> start) & (((1 << (end - start + 1)) - 1)))
			
typedef uint16_t table_idx_t;

static inline void va_table_indices(virt_addr_t va, table_idx_t& pm, table_idx_t& pdp, table_idx_t& pd, table_idx_t& pt)
{
	pm = BITS(va, 39, 47);
	pdp = BITS(va, 30, 38);
	pd = BITS(va, 21, 29);
	pt = BITS(va, 12, 20);
}

/* On 64-bit x86, the lower bits 0..6 inclusive and 8 have the same flag meaning
 * at all levels in the page table. Bit 7 is Page Size (PS) in higher levels, and
 * Page Attribute Table (PAT) at the lowest (PT) level. PAT is bit 12 at higher
 * levels. This picture is useful: https://wiki.osdev.org/File:64-bit_page_tables2.png
 * Leave this at namespace scope, not inside GenericPageTableEntry, so that ORing
 * together doesn't get noisy. It's a pity we can't "using" a class-level definition
 * to pull these into scope. */
enum PageTableEntryFlags {
	// PTE
	PTE_PRESENT		= 1<<0,
	PTE_WRITABLE	= 1<<1,
	PTE_ALLOW_USER	= 1<<2,
	PTE_WRITE_THROUGH	= 1<<3,
	PTE_CACHE_DISABLED	= 1<<4,
	PTE_ACCESSED	= 1<<5,
	PTE_DIRTY		= 1<<6,
	PTE_HUGE		= 1<<7,
	PTE_GLOBAL		= 1<<8,
};

struct GenericPageTableEntry {

	union {
		uint64_t bits;
	};
	
	inline phys_addr_t base_address() const {
		return bits & ~0xfff;
	}

	inline void base_address(phys_addr_t addr) {
		bits &= 0xfff;
		bits |= addr & ~0xfff;
	}

	inline uint16_t flags() const {
		return bits & 0xfff;
	}

	inline void flags(uint16_t flags) {
		bits &= ~0xfff;
		bits |= flags & 0xfff;
	}
	
	inline bool get_flag(PageTableEntryFlags mask) const { return !!(flags() & mask); }
	inline void set_flag(PageTableEntryFlags mask, bool v) {
		if (!v) {
			flags(flags() & ~mask); 
		} else {
			flags(flags() | mask);
		}
	}
	
	inline bool present() const { return get_flag(PageTableEntryFlags::PTE_PRESENT); }
	inline void present(bool v) { set_flag(PageTableEntryFlags::PTE_PRESENT, v); }
	
	inline bool writable() const { return get_flag(PageTableEntryFlags::PTE_WRITABLE); }
	inline void writable(bool v) { set_flag(PageTableEntryFlags::PTE_WRITABLE, v); }

	inline bool user() const { return get_flag(PageTableEntryFlags::PTE_ALLOW_USER); }
	inline void user(bool v) { set_flag(PageTableEntryFlags::PTE_ALLOW_USER, v); }

	inline bool huge() const { return get_flag(PageTableEntryFlags::PTE_HUGE); }
	inline void huge(bool v) { set_flag(PageTableEntryFlags::PTE_HUGE, v); }
} __packed;

struct PML4TableEntry : GenericPageTableEntry {
} __packed;

struct PDPTableEntry : GenericPageTableEntry {
} __packed;

struct PDTableEntry : GenericPageTableEntry {
} __packed;

struct PTTableEntry : GenericPageTableEntry {
} __packed;

} /* end namespace x86 */
} /* end namespace arch */
} /* end namespace infos */
