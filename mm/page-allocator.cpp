/* SPDX-License-Identifier: MIT */

/*
 * mm/page-allocator.cpp
 *
 * InfOS
 * Copyright (C) University of Edinburgh 2016.  All Rights Reserved.
 *
 * Tom Spink <tspink@inf.ed.ac.uk>
 */
#include <infos/mm/page-allocator.h>
#include <infos/mm/mm.h>
#include <infos/util/string.h>
#include <infos/util/lock.h>
#include <infos/util/cmdline.h>

extern char _IMAGE_START, _IMAGE_END;
extern char _STACK_START, _STACK_END;
extern char _HEAP_START;

using namespace infos::mm;
using namespace infos::kernel;
using namespace infos::util;

ComponentLog infos::mm::pgalloc_log(syslog, "pgalloc");

static bool do_self_test;

RegisterCmdLineArgument(PageAllocDebug, "pgalloc.debug")
{
	if (strncmp(value, "1", 1) == 0)
	{
		pgalloc_log.enable();
	}
	else
	{
		pgalloc_log.disable();
	}
}

RegisterCmdLineArgument(PageAllocSelfTest, "pgalloc.self-test")
{
	if (strncmp(value, "1", 2) == 0)
	{
		do_self_test = true;
	}
	else
	{
		do_self_test = false;
	}
}

PageAllocator::PageAllocator(MemoryManager &mm) : Allocator(mm), _pf_descriptors(NULL)
{
}

bool PageAllocator::setup_pf_descriptors()
{
	// Store the total number of frames in the system (ask our owner, the MemoryManager).
	_nr_frames = owner()._last_pfn + 1;

	// Calculate the total size of the page descriptor array.
	uint64_t pd_size = _nr_frames * sizeof(FrameDescriptor);

	// Use the start of the heap area to store the descriptor array.
	_pf_descriptors = (FrameDescriptor *)&_HEAP_START;
	mm_log.messagef(LogLevel::DEBUG, "Allocating %lu frame descriptors (%lu kB)", _nr_frames, KB(pd_size));

	// Make sure the descriptors will fit in this region of memory
	const PhysicalMemoryBlock *pmb = owner().lookup_phys_block(
		kva_to_pa((virt_addr_t)_pf_descriptors));
	if (!pmb)
	{
		return false;
	}

	// TODO: Actually check this assertion holds, using the size of the physical memory block.

	// Initialise the descriptors, by zeroing them all.
	bzero(_pf_descriptors, _nr_frames * sizeof(FrameDescriptor));

	return true;
}

bool PageAllocator::init()
{
	// The page allocation algorithm must be specified before initialising
	// the page allocator.
	if (!_allocator_algorithm)
	{
		mm_log.message(LogLevel::ERROR, "Page allocator algorithm not registered");
		return false;
	}

	// Prepare the frame descriptor array
	if (!setup_pf_descriptors())
	{
		return false;
	}

	// Initialise the page allocator algorithm
	mm_log.messagef(LogLevel::INFO, "Initialising allocator algorithm '%s'", _allocator_algorithm->name());
    mm_log.messagef(LogLevel::INFO, "Page Allocator: total=%lu", _nr_frames);
    if (!_allocator_algorithm->init(_pf_descriptors, _nr_frames))
	{
		mm_log.message(LogLevel::ERROR, "Allocator failed to initialise");
		return false;
	}

	uint64_t nr_present_frames, nr_free_frames;

	// Loop through available physical memory blocks, and update the corresponding frame descriptors.
	nr_present_frames = 0;
	for (unsigned int i = 0; i < owner()._nr_phys_mem_blocks; i++)
	{
		const PhysicalMemoryBlock &pmb = owner()._phys_mem_blocks[i];

		if (pmb.type == MemoryType::NORMAL)
		{
			nr_present_frames += pmb.nr_frames;
			for (pfn_t pfn = pmb.base_pfn; pfn < (pmb.base_pfn + pmb.nr_frames); pfn++)
			{
				_pf_descriptors[pfn].type = FrameDescriptorType::AVAILABLE;
			}

            _allocator_algorithm->insert_range(&_pf_descriptors[pmb.base_pfn], pmb.nr_frames);
        }
	}

    nr_free_frames = nr_present_frames;

	// Reserve page zero.  We can do without it.
    mm_log.messagef(LogLevel::INFO, "Reserving page zero");
    nr_free_frames -= reserve_range(0, 1);

    // Reserve initial page table pages
    mm_log.messagef(LogLevel::INFO, "Reserving initial page table pages");
    nr_free_frames -= reserve_range(1, 6);

	// Now, reserve the kernel's pages
	// Reserve the range of pages corresponding to the kernel image
	// pfn_t image_start_pfn = pa_to_pfn((phys_addr_t)&_IMAGE_START); // _IMAGE_START is a PA
	// pfn_t image_last_pfn = pa_to_pfn((phys_addr_t)&_IMAGE_END);	   // _IMAGE_END is a PA
	// nr_free_frames -= reserve_range(image_start_pfn, image_last_pfn - image_start_pfn + 1);
	
	// Reserve the range of pages corresponding to the kernel stack
	// pfn_t stack_start_pfn = pa_to_pfn(kva_to_pa((virt_addr_t)&_STACK_START)); // _STACK_START is a VA
	// pfn_t stack_last_pfn = pa_to_pfn(kva_to_pa((virt_addr_t)&_STACK_END));	  // _STACK_END is a VA
	// nr_free_frames -= reserve_range(stack_start_pfn, stack_last_pfn - stack_start_pfn + 1);

	// Reserve the range of frames corresponding to the kernel heap, which is
	// actually just the frames descriptors.
	// pfn_t heap_start_pfn = pa_to_pfn(kva_to_pa((virt_addr_t)&_HEAP_START)); // _HEAP_START is a VA
	// nr_free_frames -= reserve_range(heap_start_pfn, ((_nr_frames * sizeof(FrameDescriptor)) >> 12) + 1);

	// Reserve the whole range from kernel image start to kernel heap end, which eliminates the
	// problem of overlapping insertions and makes the remove_range implementation more efficient
	pfn_t image_start_pfn = pa_to_pfn((phys_addr_t)&_IMAGE_START); // _IMAGE_START is a PA
	nr_free_frames -= reserve_range(image_start_pfn, ((_nr_frames * sizeof(FrameDescriptor)) >> 12) + 1);

	mm_log.messagef(LogLevel::INFO, "Page Allocator: total=%lu, present=%lu, free=%lu (%u MB)", _nr_frames, nr_present_frames, nr_free_frames, MB(nr_free_frames << 12));

	// Now, initialise the page allocation algorithm.

	if (do_self_test)
	{
		if (!self_test())
		{
			mm_log.message(LogLevel::FATAL, "Allocator self-test failed!");
			arch_abort();
		}
	}

	return true;
}

uint64_t PageAllocator::reserve_range(pfn_t start, uint64_t nr_frames)
{
    for (pfn_t pfn = start; pfn < start + nr_frames; pfn++)
	{
		_pf_descriptors[pfn].type = FrameDescriptorType::RESERVED;
    }

    _allocator_algorithm->remove_range(&_pf_descriptors[start], nr_frames);

	return nr_frames;
}

/**
 * Allocates 2^order contiguous frames
 * @param order The power of two of the number of frames to allocate
 * @return Returns a pointer to an array of frame descriptors representing the new allocation, or NULL if allocation
 * failed.
 */
FrameDescriptor *PageAllocator::allocate(int order)
{
	// Call into the algorithm to actually allocate the pages.
	if (!_allocator_algorithm)
		return NULL;

	UniqueLock<Mutex> l(_mtx);
	FrameDescriptor *pfdescr = _allocator_algorithm->allocate(order);

	// Double check that all the pages are marked as available, and
	// mark them as allocated.
	for (unsigned int i = 0; i < (1u << order); i++)
	{
		assert(pfdescr[i].type == FrameDescriptorType::AVAILABLE);
		pfdescr[i].type = FrameDescriptorType::ALLOCATED;
	}

	pgalloc_log.messagef(LogLevel::DEBUG, "alloc: order=%d, pfdescr=%p (%lx)", order, pfdescr, pfdescr_to_pa(pfdescr));
	return pfdescr;
}

/**
 * Frees 2^order contiguous frames
 * @param pgdscr A pointer to an array of 2^order frame descriptors (contiguous)
 * @param order The power of two of the number of frames to free
 */
void PageAllocator::free(FrameDescriptor *pfdescr, int order)
{
	// Call into the algorithm to actually free the pages.
	if (_allocator_algorithm)
	{
		UniqueLock<Mutex> l(_mtx);
		_allocator_algorithm->free(pfdescr, order);

		// Double-check that all the pages were allocated, and mark them as available.
		for (unsigned int i = 0; i < (1u << order); i++)
		{
			assert(pfdescr[i].type == FrameDescriptorType::ALLOCATED);
			pfdescr[i].type = FrameDescriptorType::AVAILABLE;
		}

		pgalloc_log.messagef(LogLevel::DEBUG, "free: order=%d, pfdescr=%p (%lx)", order, pfdescr, pfdescr_to_pa(pfdescr));
	}
}

const FrameDescriptor *PageAllocator::alloc_zero_frame()
{
	const FrameDescriptor *pfdescr = allocate(0);
	if (!pfdescr)
		return NULL;

	void *ptr = (void *)pfdescr_to_vpa(pfdescr);
	pzero(ptr);

	return pfdescr;
}

bool PageAllocator::self_test()
{
	assert(_allocator_algorithm);

	mm_log.messagef(LogLevel::IMPORTANT, "PAGE ALLOCATOR SELF TEST - BEGIN");
	mm_log.messagef(LogLevel::IMPORTANT, "------------------------");

	mm_log.messagef(LogLevel::INFO, "* INITIAL STATE");
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(1) ALLOCATING ONE PAGE");
	auto p = _allocator_algorithm->allocate(0);
	if (!p)
	{
		mm_log.messagef(LogLevel::ERROR, "Allocator did not allocate page");
		return false;
	}

	mm_log.messagef(LogLevel::INFO, "ALLOCATED PFN: %p", pfdescr_to_pfn(p));
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(2) FREEING ONE PAGE");
	_allocator_algorithm->free(p, 0);

	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(3) ALLOCATING TWO CONTIGUOUS PAGES");
	p = _allocator_algorithm->allocate(1);
	mm_log.messagef(LogLevel::INFO, "ALLOCATED PFN: %p", pfdescr_to_pfn(p));

	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(4) FREEING TWO CONTIGUOUS PAGES");
	_allocator_algorithm->free(p, 1);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(5) OVERLAPPING ALLOCATIONS");
	auto p0 = _allocator_algorithm->allocate(0);
	auto p1 = _allocator_algorithm->allocate(1);
	_allocator_algorithm->free(p0, 0);
	auto p2 = _allocator_algorithm->allocate(0);
	_allocator_algorithm->free(p1, 1);
	_allocator_algorithm->free(p2, 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(6) MULTIPLE ALLOCATIONS, RANDOM ORDER FREE");
	auto p00 = _allocator_algorithm->allocate(0);
	auto p01 = _allocator_algorithm->allocate(0);
	auto p02 = _allocator_algorithm->allocate(0);
	auto p03 = _allocator_algorithm->allocate(2);
	auto p04 = _allocator_algorithm->allocate(0);
	auto p05 = _allocator_algorithm->allocate(0);
	auto p06 = _allocator_algorithm->allocate(0);
	auto p07 = _allocator_algorithm->allocate(0);
	mm_log.messagef(LogLevel::INFO, "* AFTER ALLOCATION");
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "  FREE %p", pfdescr_to_pfn(p05));
	_allocator_algorithm->free(p05, 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "  FREE %p", pfdescr_to_pfn(p03));
	_allocator_algorithm->free(p03, 2);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "  FREE %p", pfdescr_to_pfn(p06));
	_allocator_algorithm->free(p06, 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "  FREE %p", pfdescr_to_pfn(p07));
	_allocator_algorithm->free(p07, 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "  FREE %p", pfdescr_to_pfn(p02));
	_allocator_algorithm->free(p02, 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "  FREE %p", pfdescr_to_pfn(p01));
	_allocator_algorithm->free(p01, 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "  FREE %p", pfdescr_to_pfn(p00));
	_allocator_algorithm->free(p00, 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "  FREE %p", pfdescr_to_pfn(p04));
	_allocator_algorithm->free(p04, 0);

	mm_log.messagef(LogLevel::INFO, "* AFTER RANDOM ORDER FREEING");
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(7) RESERVING PAGE 0x4d80 and 0x4d84");
    _allocator_algorithm->remove_range(pfn_to_pfdescr(0x4d80), 1);
	_allocator_algorithm->remove_range(pfn_to_pfdescr(0x4d84), 1);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(8) FREEING RESERVED PAGE 0x4d84");
	_allocator_algorithm->free(pfn_to_pfdescr(0x4d84), 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "(9) FREEING RESERVED PAGE 0x4d80");
	_allocator_algorithm->free(pfn_to_pfdescr(0x4d80), 0);
	_allocator_algorithm->dump_state();

	mm_log.messagef(LogLevel::INFO, "------------------------");
	mm_log.messagef(LogLevel::INFO, "PAGE ALLOCATOR SELF TEST - COMPLETE");

	return true;
}

void PageAllocatorAlgorithm::dump_state() const
{
	pgalloc_log.messagef(LogLevel::WARNING, "dump_state() not implemented in allocation algorithm");
}
