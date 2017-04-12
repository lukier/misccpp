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
 * Linux I2C based low level transport.
 * ****************************************************************************
 */
#include <string>
#include <chrono>
#include <stdexcept>
#include <cstring>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <misccpp/lowlevelcom/transport/transport_i2c.hpp>

llc::transport::lowlevel::I2C::I2C(const char* dev_path, bool addr10b) : port_fd(-1)
{
    port_fd = ::open(dev_path, O_RDWR);
    if(port_fd < 0) return;
    
    unsigned long tenbit = (addr10b == true ? 1 : 0);
    if(::ioctl(port_fd, I2C_TENBIT, &tenbit) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return;
    }
}

llc::transport::lowlevel::I2C::~I2C()
{
    if(port_fd != -1)
    {
        ::close(port_fd);
    }
}

// TODO: what about RDWR?
    
llc::Error llc::transport::lowlevel::I2C::transmit(const void* ptr, std::size_t len, int timeout)
{
    if(port_fd < 0) { return Error::HardwareError; }
    
    if(write(port_fd, ptr, len) == (ssize_t)len)
    {
        return Error::OK;
    }
    else
    { 
        return Error::HardwareError; 
    }
}

llc::Error llc::transport::lowlevel::I2C::transmitStart(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    if(port_fd < 0) { return Error::HardwareError; }
    
    const unsigned long timeout_10ms = timeout / 10;
    const unsigned long i2caddr = node_id; // NOTE NodeID is I2C address
    
    if(::ioctl(port_fd, I2C_TIMEOUT, &timeout_10ms) < 0)
    {
        return Error::HardwareError;
    }
    
    if(::ioctl(port_fd, I2C_SLAVE, &i2caddr) < 0)
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}

llc::Error llc::transport::lowlevel::I2C::transmitReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::I2C::transmitComplete(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::I2C::receive(void* ptr, std::size_t len, int timeout)
{
    // TODO FIXME RawIO design will call this twice (PayloadLengthT)! Need to redesign.
    
    if(port_fd < 0) { return Error::HardwareError; }
    
    if(read(port_fd, ptr, len) == (ssize_t)len)
    {
        return Error::OK;
    }
    else
    { 
        return Error::HardwareError; 
    }
}

llc::Error llc::transport::lowlevel::I2C::receiveStart(int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::I2C::receiveReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::I2C::receiveComplete(int timeout)
{
    return Error::OK;
}
