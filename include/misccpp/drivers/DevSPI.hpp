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
 * Simple C++ interface to spi-dev.
 * ****************************************************************************
 */

#ifndef DRIVERS_DEV_SPI_HPP
#define DRIVERS_DEV_SPI_HPP

#include <cstdint>
#include <chrono>

namespace drivers
{

/**
 * SPI Dev Interface.
 * NOTE: Blocking, no timeouts.
 */    
class DevSPI
{
public:
    DevSPI();
    virtual ~DevSPI();
    
    bool open(const char* spidev, uint32_t freq = 1000000, uint8_t bpw = 8, bool cpha = true, bool cpol = true, bool lsb_first = true);
    bool isOpen();
    void close();

    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool read(void* dst, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return readImpl(dst, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool write(const void* src, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return writeImpl(src, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }

    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool readWrite(const void* data_out, void* data_in, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return readWriteImpl(data_out, data_in, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
private:
    bool readImpl(void* dst, std::size_t count, const uint32_t timeout_ms);
    bool writeImpl(const void* src, std::size_t count, const uint32_t timeout_ms);
    bool readWriteImpl(const void* data_out, void* data_in, std::size_t count, const uint32_t timeout_ms);
    int port_fd;
};
    
}

#endif // DRIVERS_DEV_SPI_HPP
