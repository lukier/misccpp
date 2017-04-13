/**
 * ****************************************************************************
 * Copyright (c) 2017, Robert Lukierski.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * ****************************************************************************
 * Interface to create packet transports from low level transports.
 * ****************************************************************************
 */
#ifndef MISCCPP_LOCK_FREE_FIFO_HPP
#define MISCCPP_LOCK_FREE_FIFO_HPP

#include <cstddef>
#include <atomic>
#include <type_traits>

template <typename T>
class lock_free_fifo
{
public:
    typedef T value_type;
    typedef std::size_t size_type;
    
    lock_free_fifo() : ring_(nullptr), head_(0), tail_(0), buffer_size_(0) { }
    
    lock_free_fifo(T* ring, size_type ring_size) : ring_(ring), head_(0), tail_(0), buffer_size_(ring_size) {}
    
    inline void clear()
    {
        head_ = 0;
        tail_ = 0;
    }

    inline bool push_front(const T& value)
    {
        size_type head = head_.load(std::memory_order_relaxed);
        
        size_type next_head = next(head);
        if(next_head == tail_.load(std::memory_order_acquire))
        {
            // FULL
            return false;
        }
        
        ring_[head] = std::move(value);
        
        head_.store(next_head, std::memory_order_release);
        
        return true;
    }
    
    inline bool pop_back(T& value)
    {
        size_type tail = tail_.load(std::memory_order_relaxed);
        
        if(tail == head_.load(std::memory_order_acquire))
        {
            return false;
        }
        else
        {
            value = std::move(ring_[tail]); 
            
            tail_.store(next(tail), std::memory_order_release);
            
            return true;
        }
    }
    
    inline bool empty() const
    {
        size_type tail = tail_.load(std::memory_order_relaxed);
        
        if(tail == head_.load(std::memory_order_acquire))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    inline bool full() const 
    { 
        size_type head = head_.load(std::memory_order_relaxed);
        
        size_type next_head = next(head);
        if(next_head == tail_.load(std::memory_order_acquire))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    inline size_type capacity() const { return buffer_size_; }
    
    inline void reconfigure(T* ring, size_type bsize, size_type already_there = 0)
    {
        ring_ = ring;
        buffer_size_ = bsize;
        head_ = already_there;
        tail_ = 0;
    }
    
private:    
    inline size_type next(size_type current) const { return (current + 1) % (buffer_size_+1); }
    
    T* ring_;
    std::atomic<size_type> head_, tail_;
    size_type buffer_size_;
};

#endif // MISCCPP_LOCK_FREE_FIFO_HPP
