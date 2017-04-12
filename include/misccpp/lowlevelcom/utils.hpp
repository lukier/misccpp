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
 * Defines, common traits & types and utilities.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_UTILS_HPP
#define LOWLEVELCOM_UTILS_HPP

#include <cstdint>
#include <cstddef>
#include <utility>

namespace llc
{

enum class Error : uint8_t
{
    OK = 0,
    Unsupported,
    Timeout,
    OutOfMemory,
    CRCMismatch,
    FramingError,
    ChannelError,
    HardwareError,
    OutOfData
};

enum class AccessT : uint8_t
{
    R,
    W,
    RW
};

typedef uint16_t NodeIDT;
typedef uint32_t PayloadLengthT;
typedef uint8_t ChannelIDT;

namespace internal
{
    template<bool v>
    struct NodeIDSize
    {
        static constexpr std::size_t size = sizeof(NodeIDT);
    };
    
    template<>
    struct NodeIDSize<false>
    {
        static constexpr std::size_t size = 0;
    };
    
    template<bool v>
    struct ChannelIDSize
    {
        static constexpr std::size_t size = sizeof(ChannelIDT);
    };
    
    template<>
    struct ChannelIDSize<false>
    {
        static constexpr std::size_t size = 0;
    };
    
    struct NoMutex
    {
        void lock() { }
        void unlock() { }
    };
    
    template<typename _Mutex>
    class lock_guard
    {
    public:
        typedef _Mutex mutex_type;
        
        explicit lock_guard(mutex_type& __m) : _M_device(__m) { _M_device.lock(); }
        ~lock_guard() { _M_device.unlock(); }
        
        lock_guard(const lock_guard&) = delete;
        lock_guard& operator=(const lock_guard&) = delete;
        
    private:
        mutex_type&  _M_device;
    };

    struct NoCRC
    {
        static constexpr bool crc_enabled = false;
        typedef uint32_t crc_type;
        static constexpr std::size_t crc_suffix_size = 0;
        static inline crc_type calculateCRC(std::size_t count, const void* buffer) { return 0; }
    };
}

struct VarArray
{
    VarArray() : Size(0), Buffer(0) { }
    
    std::size_t Size;
    void* Buffer;
};

#ifndef LLC_HANDLE_ERROR
#define LLC_HANDLE_ERROR(RC) (RC)
#endif // LLC_HANDLE_ERROR

}

#endif // LOWLEVELCOM_UTILS_HPP
