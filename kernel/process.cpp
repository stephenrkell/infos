/* SPDX-License-Identifier: MIT */

/*
 * kernel/process.cpp
 *
 * InfOS
 * Copyright (C) University of Edinburgh 2016.  All Rights Reserved.
 *
 * Tom Spink <tspink@inf.ed.ac.uk>
 */
#include <infos/kernel/process.h>

using namespace infos::kernel;

Process::Process(const util::String& name, bool kernel_process,
	Thread::thread_proc_t entry_point, fs::File *file /* = nullptr */)
	: _name(name), _kernel_process(kernel_process), _terminated(false), _vma(), _file(file)
{
	// Initialise the VMA by installing the default kernel mapping.
	_vma.install_default_kernel_mapping();

	// Create the main thread.
	_main_thread = &create_thread(kernel_process ? ThreadPrivilege::Kernel : ThreadPrivilege::User, entry_point, "main");
}

Process::~Process()
{
	// All threads /should/ be stopped by this point.
	for (const auto& thread : _threads) {
		assert(thread->state() == SchedulingEntityState::STOPPED);
		delete thread;
	}
	// We need to delete the handle on our executable file, if we have one
	// FIXME: if not kernel-mode, should definitely have a file
	if (_file) delete _file;
}

void Process::start()
{
	// Start the main thread.
	main_thread().start();
}

void Process::terminate(int rc)
{
	_terminated = true;
	_state_changed.trigger();

	for (const auto& thread : _threads) {
		thread->stop();
	}
}

Thread& Process::create_thread(ThreadPrivilege::ThreadPrivilege privilege, Thread::thread_proc_t entry_point,
        const util::String& name, SchedulingEntityPriority::SchedulingEntityPriority priority)
{
	// A kernel process can NEVER have a user thread.
	assert(!(kernel_process() && privilege == ThreadPrivilege::User));

	Thread *new_thread = new Thread(*this, privilege, entry_point, priority, name);
	_threads.append(new_thread);

	return *new_thread;
}
