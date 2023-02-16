/* SPDX-License-Identifier: MIT */

/*
 * mm/simple-page-alloc.cpp
 *
 * InfOS
 * Copyright (C) University of Edinburgh 2016.  All Rights Reserved.
 *
 * Tom Spink <tspink@inf.ed.ac.uk>
 */
#include <infos/mm/page-allocator.h>
#include <infos/mm/mm.h>
#include <infos/kernel/kernel.h>
#include <infos/util/list.h>

using namespace infos::kernel;
using namespace infos::mm;
using namespace infos::util;

/**
 * A simple page allocation algorithm. As with PageAllocator, it is
 * misnamed because it really allocates frames.
 */
class SimplePageAllocatorAlgorithm : public PageAllocatorAlgorithm
{
private:
	FrameDescriptor *_pfdescr_base;
	uint64_t _nr_pfdescrs;

public:
	bool init(FrameDescriptor *pf_descriptors, uint64_t nr_pf_descriptors) override
	{
		mm_log.messagef(LogLevel::DEBUG, "Simple Page Allocator online");
		_pfdescr_base = pf_descriptors;
		_nr_pfdescrs = nr_pf_descriptors;

		return true;
	}

	FrameDescriptor *allocate(int order) override
	{
		const int nr_pages = (1 << order);
		for (uint64_t idx = 0; idx < _nr_pfdescrs; idx++)
		{
			bool found = true;
			for (uint64_t subidx = idx; subidx < idx + nr_pages; subidx++)
			{
				if (_pfdescr_base[subidx].type != FrameDescriptorType::AVAILABLE)
				{
					found = false;
					idx = subidx;
					break;
				}
			}

			if (found)
			{
				return &_pfdescr_base[idx];
			}
		}

		return NULL;
	}

	void free(FrameDescriptor *pfdescr, int order) override
	{
		FrameDescriptor *base = pfdescr;
		for (unsigned int i = 0; i < ((unsigned int)1 << (unsigned int)order); i++)
		{
			// Causes the self test to fail because we don't actually allocate the pages
			// However, this is fine, because the self test doesn't really make sense in 
			// the context of the simple page allocator, which has no internal state
			assert(base[i].type == FrameDescriptorType::ALLOCATED);
		}
	}

	virtual void insert_range(FrameDescriptor *start, uint64_t count) override
	{
		mm_log.messagef(LogLevel::DEBUG, "Inserting available frames from %lx -- %lx", sys.mm().pgalloc().pfdescr_to_pfn(start), sys.mm().pgalloc().pfdescr_to_pfn(start + count));
	}

	virtual void remove_range(FrameDescriptor *start, uint64_t count) override
	{
		mm_log.messagef(LogLevel::DEBUG, "Removing available frames from %lx -- %lx", sys.mm().pgalloc().pfdescr_to_pfn(start), sys.mm().pgalloc().pfdescr_to_pfn(start + count));
	}

	const char *name() const override { return "simple"; }

	void dump_state() const override
	{
	}
};

RegisterPageAllocatorAlgorithm(SimplePageAllocatorAlgorithm);
