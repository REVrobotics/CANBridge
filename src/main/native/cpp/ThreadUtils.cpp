/*
 * Copyright (c) 2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "utils/ThreadUtils.h"

namespace rev {
namespace usb {
namespace utils {

#if defined(__unix__)
// pthreads TODO:

bool SetThreadPriority(std::thread* thread, ThreadPriority p) {
    (void)thread;
    (void)p;
    return false;
}

bool SetCurrentThreadPriority(std::thread* thread, ThreadPriority p) {
    (void)thread;
    (void)p;
    return false;
}

ThreadPriority GetThreadPriority(std::thread* thread) {
    (void)thread;
    return ThreadPriority::PriorityError;
}

#elif defined(_WIN32)
// Windows API threads

#include <windows.h>

static int ToNativePriority(ThreadPriority p) {
    int nativePriority;

    switch (p) {
    case ThreadPriority::Low:
        nativePriority = THREAD_PRIORITY_LOWEST;
        break;
    case ThreadPriority::BelowNormal:
        nativePriority = THREAD_PRIORITY_BELOW_NORMAL;
        break;
    case ThreadPriority::Normal:
        nativePriority = THREAD_PRIORITY_NORMAL;
        break;
    case ThreadPriority::AboveNormal:
        nativePriority = THREAD_PRIORITY_ABOVE_NORMAL;
        break;
    case ThreadPriority::High:
        nativePriority = THREAD_PRIORITY_HIGHEST;
        break;
    case ThreadPriority::PriorityMax:
        nativePriority = THREAD_PRIORITY_TIME_CRITICAL;
        break;
    default:
        nativePriority = THREAD_PRIORITY_NORMAL;
        break;
    }

    return nativePriority;
}

static ThreadPriority FromNativePriority(int nativePriority) {
    ThreadPriority pri;

    switch (nativePriority) {
    case THREAD_PRIORITY_LOWEST:
        pri = ThreadPriority::Low;
        break;
    case THREAD_PRIORITY_BELOW_NORMAL:
        pri = ThreadPriority::BelowNormal;
        break;
    case THREAD_PRIORITY_NORMAL:
        pri = ThreadPriority::Normal;
        break;
    case THREAD_PRIORITY_ABOVE_NORMAL:
        pri = ThreadPriority::AboveNormal;
        break;
    case THREAD_PRIORITY_HIGHEST:
        pri = ThreadPriority::High;
        break;
    case THREAD_PRIORITY_TIME_CRITICAL:
        pri = ThreadPriority::PriorityMax;
        break;
    default:
        pri = ThreadPriority::Normal;
        break;
    }

    return pri;    
}

// For some reason VC++ complains about using global namespace for ::SetThreadPriority
// but GCC complains if we specify the namespace for the function directly. Compromise
// is to change the namespace when declaring the functions for _WIN32 using VC++
} // namespace utils


bool utils::SetThreadPriority(std::thread* thread, utils::ThreadPriority p) {
    return (bool)SetThreadPriority(thread->native_handle(), ToNativePriority(p));
}

bool utils::SetCurrentThreadPriority(ThreadPriority p) {
    return (bool)SetThreadPriority(GetCurrentThread(), ToNativePriority(p));
}

utils::ThreadPriority utils::GetThreadPriority(std::thread* thread) {
    return utils::FromNativePriority(GetThreadPriority(thread->native_handle()));
}

namespace utils {

#else

bool SetThreadPriority(std::thread& thread, ThreadPriority p) {
    (void)thread;
    (void)p;
    return false;
}

ThreadPriority ThreadPriority(std::thread& thread) {
    (void)thread;
    return ThreadPriority::PriorityError;
}

#endif

} // namespace utils
} // namespace usb
} // namespace rev
