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
 * Linux i2c-dev based transport.
 * ****************************************************************************
 */
#include <string>
#include <chrono>
#include <stdexcept>
#include <cstring>

#include <misccpp/drivers/DevI2C.hpp>

#include <misccpp/lowlevelcom/transport/transport_i2c.hpp>

template<bool MultiChannel>
llc::transport::I2CIO<MultiChannel>::I2CIO(const char* port, bool addr10b) : tx_i2c_addr(0), pimpl(new drivers::DevI2C())
{
    pimpl->open(port, (addr10b == true ? drivers::DevI2C::AddressSupport::bits10 : drivers::DevI2C::AddressSupport::bits7));
}

template<bool MultiChannel>
llc::transport::I2CIO<MultiChannel>::~I2CIO()
{
    
}

template<bool MultiChannel>
llc::Error llc::transport::I2CIO<MultiChannel>::receiveImpl(message_t& msg_out, const int timeout_ms)
{
    // msg_out must contain info (length) beforehand for packet based IO
    if(!msg_out) { return Error::OutOfMemory; }
    
    if(!pimpl->setSlaveAddress(tx_i2c_addr))
    {
        return Error::HardwareError;
    }
    
    msg_out.setNodeID(tx_i2c_addr);
    
    if(!pimpl->read(msg_out.buf.get(), message_t::PrefixRequired + msg_out.len, std::chrono::milliseconds(timeout_ms)))
    {
        return Error::HardwareError;
    }
    
    tx_i2c_addr = 0;
    
    return Error::OK;
}

template<bool MultiChannel>
llc::Error llc::transport::I2CIO<MultiChannel>::transmitImpl(const message_t& msg_out, const int timeout_ms)
{
    tx_i2c_addr = msg_out.getNodeID();
    
    if(!pimpl->setSlaveAddress(tx_i2c_addr))
    {
        return Error::HardwareError;
    }
    
    if(!pimpl->write(msg_out.buf.get(), message_t::PrefixRequired + msg_out.len, std::chrono::milliseconds(timeout_ms)))
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}


template class llc::transport::I2CIO<true>;
template class llc::transport::I2CIO<false>;
