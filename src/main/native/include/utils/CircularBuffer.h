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

#pragma once

#include <iostream>
#include <vector>

namespace utils {

template<typename T> class CircularBuffer {
public:
    CircularBuffer(size_t size) : size(size), buffer(size) { }
    
    CircularBuffer(CircularBuffer<T>&& rhs) = default;
    CircularBuffer<T>& operator=(CircularBuffer<T>& rhs) = default;

    bool IsEmpty() { return count == 0 || buffer.size() == 0; }
    bool IsFull() { return count == size; }
    size_t GetCount() { return count; }

    // Add to the bugger
    bool Add(const T& t) {
        if (IsFull()) {
            return false;
        } else {
            buffer[tail] = t;
            count++; 
            tail = (tail == size - 1) ? 0 : tail + 1;
            return true;
        }
    }

    // Remove a set number from the head of the buffer 
    // Buffer is FIFO
    bool Remove(T& t) {
        if (IsEmpty()) {
            return false;
        } else {
            t = buffer[head];
            count--;
            head = (head == size - 1) ? 0 : head + 1;
            return true;
        }
    } 
    
 
private:
    size_t size;
    std::vector<T> buffer;
    size_t head = 0; // Oldest element
    size_t tail = 0; // Youngest element
    size_t count = 0;
};


} // namespace utils