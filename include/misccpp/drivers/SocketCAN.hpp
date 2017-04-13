/**
 * ****************************************************************************
 * Copyright (c) 2016, Robert Lukierski.
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
 * Simple C++ SocketCAN interface.
 * ****************************************************************************
 */

#ifndef DRIVERS_SOCKET_CAN_HPP
#define DRIVERS_SOCKET_CAN_HPP

#include <cstdint>
#include <chrono>

namespace drivers
{
    
/**
 * SocketCAN Interface.
 */
class SocketCAN
{
public:    
    SocketCAN();
    virtual ~SocketCAN();
    
    bool open(const char* port, bool ufd);
    bool isOpen();
    void close();
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool read(uint32_t& cid, void* dst, std::size_t& count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return readImpl(cid, dst, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool write(const uint32_t& cid, const void* src, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return writeImpl(cid, src, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
private:
    bool readImpl(uint32_t& cid, void* dst, std::size_t& count, const uint32_t timeout_ms);
    bool writeImpl(const uint32_t& cid, const void* src, std::size_t count, const uint32_t timeout_ms);
    int sock;
    bool use_fd;
};
    
}

#endif // DRIVERS_SOCKET_CAN_HPP
