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
 * Linux SPI based low level and high level packet transport.
 * ****************************************************************************
 */
#include <string>
#include <chrono>
#include <stdexcept>
#include <cstring>

#include <misccpp/drivers/DevSPI.hpp>

#include <misccpp/lowlevelcom/transport/transport_spi.hpp>

llc::transport::lowlevel::SPI::SPI(const char* dev_path, uint32_t freq, uint8_t bpw, bool cpha, bool cpol, bool lsb_first) : pimpl(new drivers::DevSPI())
{
    pimpl->open(dev_path, freq, bpw, cpha, cpol, lsb_first);
}

llc::transport::lowlevel::SPI::~SPI()
{
    
}
    
llc::Error llc::transport::lowlevel::SPI::transmit(const void* ptr, PayloadLengthT len, int timeout)
{
    if(!pimpl->write(ptr, len, std::chrono::milliseconds(timeout)))
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SPI::transmitStart(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SPI::transmitReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SPI::transmitComplete(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SPI::receive(void* ptr, PayloadLengthT len, int timeout)
{
    if(!pimpl->read(ptr, len, std::chrono::milliseconds(timeout)))
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SPI::receiveStart(PayloadLengthT& len, ChannelIDT& chan_id, NodeIDT& node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SPI::receiveReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SPI::receiveComplete(int timeout)
{
    return Error::OK;
}

// --------------------------------------------

template<bool MultiChannel, bool MultiNode>
llc::transport::SPIIO<MultiChannel,MultiNode>::SPIIO(const char* dev_path, uint32_t freq, uint8_t bpw, bool cpha, bool cpol, bool lsb_first) : pimpl(new drivers::DevSPI())
{
    pimpl->open(dev_path, freq, bpw, cpha, cpol, lsb_first);
}

template<bool MultiChannel, bool MultiNode>
llc::transport::SPIIO<MultiChannel,MultiNode>::~SPIIO()
{
    
}

template<bool MultiChannel, bool MultiNode>
llc::Error llc::transport::SPIIO<MultiChannel,MultiNode>::receiveImpl(message_t& msg_out, const int timeout_ms)
{
    // msg_out must contain info (length) beforehand for packet based IO
    if(!msg_out) { return Error::OutOfMemory; }
    
    if(!pimpl->read(msg_out.buf.get(), msg_out.len, std::chrono::milliseconds(timeout_ms)))
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}

template<bool MultiChannel, bool MultiNode>
llc::Error llc::transport::SPIIO<MultiChannel,MultiNode>::transmitImpl(const message_t& msg_out, const int timeout_ms)
{
    if(!pimpl->write(msg_out.buf.get(), msg_out.len, std::chrono::milliseconds(timeout_ms)))
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}

template class llc::transport::SPIIO<false,false>;
template class llc::transport::SPIIO<false,true>;
template class llc::transport::SPIIO<true,false>;
template class llc::transport::SPIIO<true,true>;
