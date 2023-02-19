/* SPDX-License-Identifier: MIT */
#pragma once

#include <infos/fs/exec/loader.h>
#include <infos/kernel/log.h>

namespace infos
{
	namespace fs
	{
		class File;
		
		namespace exec
		{
			enum class ELFType : uint16_t
			{
				ET_NONE = 0,
				ET_REL = 1,
				ET_EXEC = 2,
				ET_DYN = 3,
				ET_CORE = 4
			};

			struct ELF64Header
			{
				struct
				{
					union
					{
						char magic_bytes[4];
						uint32_t magic_number;
					};

					uint8_t eclass, data, version, osabi, abiversion;
					uint8_t pad[7];
				} ident __packed;

				ELFType type;
				uint16_t machine;
				uint32_t version;
				uint64_t entry_point;
				uint64_t phoff;
				uint64_t shoff;
				uint32_t flags;
				uint16_t ehsize;
				uint16_t phentsize;
				uint16_t phnum;
				uint16_t shentsize;
				uint16_t shnum;
				uint16_t shstrndx;
			} __packed;

			enum class ProgramHeaderEntryType : uint32_t
			{
				PT_NULL = 0,
				PT_LOAD = 1,
				PT_DYNAMIC = 2,
				PT_INTERP = 3,
				PT_NOTE = 4,
				PT_SHLIB = 5,
				PT_PHDR = 6
			};

			/* Don't use 'enum class' for this one, because it makes it
			 * more noisy to bitwise AND them. */
			enum ProgramHeaderEntryFlags
			{
				PF_X = 1,
				PF_W = 2,
				PF_R = 4
			};

			struct ELF64ProgramHeaderEntry
			{
				ProgramHeaderEntryType type;
				ProgramHeaderEntryFlags flags;
				uint64_t offset, vaddr, paddr;
				uint64_t filesz, memsz, align;
			};

			class ElfLoader : public Loader
			{
			public:
				ElfLoader(File& f);
				virtual ~ElfLoader() { }
				
				kernel::Process* load(const util::String& cmdline) override;
				
			private:
				File& _file;
			};
			
			extern kernel::ComponentLog elf_log;
		}		
	}
}
