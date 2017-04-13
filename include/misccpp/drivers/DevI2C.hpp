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
 * Simple C++ interface to i2c-dev.
 * ****************************************************************************
 */

#ifndef DRIVERS_DEV_I2C_HPP
#define DRIVERS_DEV_I2C_HPP

#include <cstdint>
#include <chrono>

namespace drivers
{

/**
 * I2C Dev Interface.
 * NOTE: Blocking, no timeouts.
 */    
class DevI2C
{
public:
    enum class AddressSupport
    {
        bits7,
        bits10
    };
    
    DevI2C();
    virtual ~DevI2C();
    
    bool open(const char* i2cdev, AddressSupport as = AddressSupport::bits7);
    bool isOpen();
    void close();
    
    uint32_t getSlaveAddress() const { return ouraddr; }
    bool setSlaveAddress(const uint32_t a);

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
    bool writeRead(const void* data_out, std::size_t count_out, void* data_in, std::size_t count_in, bool noStart = false, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return readWriteImpl(false, data_out, count_out, data_in, count_in, noStart, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool writeWrite(const void* data_out, std::size_t count_out, const void* data_out2, std::size_t count_out2, bool noStart = false, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return readWriteImpl(true, data_out, count_out, const_cast<void*>(data_out2), count_out2, noStart, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
private:
    bool readImpl(void* dst, std::size_t count, const uint32_t timeout_ms);
    bool writeImpl(const void* src, std::size_t count, const uint32_t timeout_ms);
    bool readWriteImpl(bool ww, const void* data_out, std::size_t count_out, void* data_in, std::size_t count_in, bool noStart, const uint32_t timeout_ms);
    uint32_t ouraddr;
    int port_fd;
};
    
}

#endif // DRIVERS_DEV_I2C_HPP
