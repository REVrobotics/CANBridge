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

// template<typename T> CircularBuffer<T>::CircularBuffer(CircularBuffer<T>&& rhs)
//     : size(std::move(rhs.size)), buffer(std::move(rhs.buffer)), head(std::move(rhs.head)), tail(std::move(rhs.tail))
// {
//     std::cout << "Move contructor" << std::endl;
// }


// template<typename T> CircularBuffer<T>& CircularBuffer<T>::operator=(CircularBuffer<T>& rhs)
// {
//     std::cout << "Copy constructor" << std::endl;
//     size = std::move(rhs.size);
//     buffer = std::move(rhs.buffer);
//     head = std::move(rhs.head);
//     tail = std::move(rhs.tail);
//     return *this;
// }

// template<typename T> T& CircularBuffer<T>::operator[](size_t index)
// {
//     return const_cast<T&>(static_cast<const CircularBuffer<T>&>(*this)[index]);
// }

// template<typename T> bool CircularBuffer<T>::Add(const T &t) {
//     if (IsFull()) {
//         return false;
//     } else {
//         auto index = (tail == size - 1) ? 0 : tail + 1;
//         buffer[index] = t;
//         tail++;
//         return true;
//     }
// }

// template<typename T> std::vector<T> CircularBuffer<T>::Remove(uint32_t amount) {
//     std::vector<T> toReturn(amount);
//     if (!IsEmpty()) {
//         while (amount > 0) {
//             toReturn.push_back(buffer[head]);
//             buffer.erase(buffer.begin() + head);
//             head = (head == size - 1) ? 0 : head + 1;
//             amount--;
//         }
//     }
//     return toReturn;
// }


} // namespace utils