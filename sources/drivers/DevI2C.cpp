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

#include <misccpp/drivers/DevI2C.hpp>

#include <string>
#include <chrono>
#include <stdexcept>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

static inline timespec timespec_from_ms(const uint32_t timeout_ms)
{
    timespec time;
    time.tv_sec = timeout_ms / 1e3;
    time.tv_nsec = (timeout_ms - (time.tv_sec * 1e3)) * 1e6;
    return time;
}

drivers::DevI2C::DevI2C() : port_fd(-1)
{

}

drivers::DevI2C::~DevI2C()
{

}

bool drivers::DevI2C::open(const char* i2cdev, AddressSupport as)
{
    port_fd = ::open(i2cdev, O_RDWR);
    if(port_fd < 0) return false;
    
    unsigned long tenbit = (as == AddressSupport::bits10 ? 1 : 0);
    if(::ioctl(port_fd, I2C_TENBIT, &tenbit) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    return true;
}

bool drivers::DevI2C::isOpen()
{
    return port_fd != -1;
}

void drivers::DevI2C::close()
{
    if(isOpen())
    {
        ::close(port_fd);
        port_fd = -1;
    }
}

bool drivers::DevI2C::setSlaveAddress(const uint32_t a)
{
    if(!isOpen()) { return false; }
    
    if(::ioctl(port_fd, I2C_SLAVE, &a) < 0)
    {
        return false;
    }
    
    ouraddr = a;
    
    return true;
}

bool drivers::DevI2C::readImpl(void* dst, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    if(::read(port_fd, dst, count) == (ssize_t)count)
    {
        return true;
    }
     
    return false;
}

bool drivers::DevI2C::writeImpl(const void* src, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    if(::write(port_fd, src, count) == (ssize_t)count)
    {
        return true;
    }
    
    return false;
}

bool drivers::DevI2C::readWriteImpl(bool ww, const void* data_out, std::size_t count_out, void* data_in, std::size_t count_in, bool noStart, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    
    // outbound
    messages[0].addr  = ouraddr;
    messages[0].flags = 0;
    messages[0].len   = count_out;
    messages[0].buf   = const_cast<uint8_t*>(static_cast<const uint8_t*>(data_out));
    
    messages[1].addr  = ouraddr;
    messages[1].flags = 0;
    if(!ww) // second is read
    {
        messages[1].flags |= I2C_M_RD;
    }
    if(noStart)
    {
        messages[1].flags |= I2C_M_NOSTART;
    }
    messages[1].len   = count_in;
    messages[1].buf   = static_cast<uint8_t*>(data_in);
    
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    
    if(::ioctl(port_fd, I2C_RDWR, &packets) < 0) 
    {
        return false;
    }
    
    return true;
}
