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