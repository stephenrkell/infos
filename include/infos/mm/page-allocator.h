/* SPDX-License-Identifier: MIT */

/*
 * include/mm/page-allocator.h
 *
 * InfOS
 * Copyright (C) University of Edinburgh 2016.  All Rights Reserved.
 *
 * Tom Spink <tspink@inf.ed.ac.uk>
 */
#pragma once

#include <infos/mm/allocator.h>
#include <infos/kernel/log.h>
#include <infos/util/lock.h>

namespace infos
{
	namespace mm
	{
		namespace FrameDescriptorType
		{
			enum FrameDescriptorType
			{
				INVALID = 0,
				RESERVED = 1,
				AVAILABLE = 2,
				ALLOCATED = 3,
			};
		}

		/* There is a FrameDescriptor for each frame of physical memory
		 * that might be available to the page allocator. (In practice
		 * we create one for every single frame of physical memory...
		 * see setup_pf_descriptors() in page-allocator.cpp.) */
		struct FrameDescriptor
		{
			FrameDescriptor *next; // XXX: currently unused! but could be used for a free list
			FrameDescriptor *prev; // XXX: currently unused! but could be used for a free list
			FrameDescriptorType::FrameDescriptorType type;
		} __aligned(16);

		class MemoryManager;
		class ObjectAllocator;

		/* The abstract base class of page allocator algorithms.
		 * 'Page allocators' are somewhat misnamed  because they really
		 * allocate physical frames, not (virtual) pages. However, they
		 * are almost always called from a VMA implementation which is
		 * managing a page table, so at the point of use they can be
		 * viewed as allocating both pages and  frames. */
		class PageAllocatorAlgorithm
		{
		public:
			virtual bool init(FrameDescriptor *pf_descriptors, uint64_t nr_pf_descriptors) = 0;

			virtual void insert_range(FrameDescriptor *start, uint64_t count) = 0;
			virtual void remove_range(FrameDescriptor *start, uint64_t count) = 0;

			virtual FrameDescriptor *allocate(int order) = 0;
			virtual void free(FrameDescriptor *base, int order) = 0;

			virtual const char *name() const = 0;

			virtual void dump_state() const;
		};

		/**
		 * The class of page allocators. As noted above for PageAllocatorAlgorithm,
		 * these are misnamed because they really allocate frames.
		 */
		class PageAllocator : Allocator
		{
			friend class MemoryManager;
			friend class ObjectAllocator;

		public:
			PageAllocator(MemoryManager &mm);

			bool init() override;

			PageAllocatorAlgorithm *algorithm() const { return _allocator_algorithm; }
			void algorithm(PageAllocatorAlgorithm &alg) { _allocator_algorithm = &alg; }

			FrameDescriptor *allocate(int order);
			void free(FrameDescriptor *pfdescr, int order);

			const FrameDescriptor *alloc_zero_frame();
			inline void free_one(FrameDescriptor *pfdescr) { return free(pfdescr, 0); }

			pfn_t pfdescr_to_pfn(const FrameDescriptor *pfdescr) const
			{
				uintptr_t offset = (uintptr_t)pfdescr - (uintptr_t)_pf_descriptors;
				offset /= sizeof(FrameDescriptor);

				return (pfn_t)offset;
			}

			phys_addr_t pfdescr_to_pa(const FrameDescriptor *pfdescr) const
			{
				return (phys_addr_t)pfn_to_pa(pfdescr_to_pfn(pfdescr));
			}

			virt_addr_t pfdescr_to_vpa(const FrameDescriptor *pfdescr) const
			{
				return (virt_addr_t)pa_to_vpa(pfdescr_to_pa(pfdescr));
			}

			virt_addr_t pfdescr_to_kva(const FrameDescriptor *pfdescr) const
			{
				return (virt_addr_t)pa_to_kva(pfdescr_to_pa(pfdescr));
			}

			FrameDescriptor *pfn_to_pfdescr(pfn_t pfn) const
			{
				if (pfn > _nr_frames)
					return NULL;
				return &_pf_descriptors[pfn];
			}

			FrameDescriptor *vpa_to_pfdescr(virt_addr_t addr) const
			{
				phys_addr_t pa = vpa_to_pa(addr);
				pfn_t pfn = pa_to_pfn(pa);
				return pfn_to_pfdescr(pfn);
			}

		private:
			uint64_t _nr_frames;
			FrameDescriptor *_pf_descriptors;
			PageAllocatorAlgorithm *_allocator_algorithm;
			util::Mutex _mtx;

			bool setup_pf_descriptors();
			bool self_test();
			uint64_t reserve_range(pfn_t start, uint64_t nr_frames);
		};

		extern infos::kernel::ComponentLog pgalloc_log;

#define RegisterPageAllocatorAlgorithm(_class) \
	static _class __pgalloc_class;    \
	__section(".pgallocptr") infos::mm::PageAllocatorAlgorithm *__pgalloc_ptr_##_class = &__pgalloc_class
	}
}
