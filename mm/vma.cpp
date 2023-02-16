/* SPDX-License-Identifier: MIT */

/*
 * mm/vma.cpp
 * 
 * InfOS
 * Copyright (C) University of Edinburgh 2016.  All Rights Reserved.
 * 
 * Tom Spink <tspink@inf.ed.ac.uk>
 */
#include <infos/mm/vma.h>
#include <infos/mm/mm.h>
#include <infos/kernel/kernel.h>
#include <infos/util/string.h>

using namespace infos::mm;
using namespace infos::kernel;
using namespace infos::util;

VMA::VMA()
{
	/* Allocate a single page to hold the root of the page table
	 * for this VMA. */
	auto pfdescr = allocate_phys(0);
	assert(pfdescr);
	
	_pgt_phys_base = sys.mm().pgalloc().pfdescr_to_pa(pfdescr);
	_pgt_virt_base = sys.mm().pgalloc().pfdescr_to_vpa(pfdescr);
}

VMA::~VMA()
{
	// TODO: Release allocations
	
}
