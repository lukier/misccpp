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
 * Linux libUSB based low level transport.
 * ****************************************************************************
 */
#include <stdexcept>

#include <misccpp/drivers/DevUSB.hpp>

#include <misccpp/lowlevelcom/transport/transport_usb.hpp>

llc::transport::lowlevel::USB::USB(uint16_t a_vid, uint16_t a_pid, uint8_t a_ep_in, uint8_t a_ep_out) : pimpl(new drivers::DevUSB())
{
    pimpl->open(a_vid, a_pid, a_ep_in, a_ep_out);
}

llc::transport::lowlevel::USB::~USB()
{
    
}

llc::Error llc::transport::lowlevel::USB::transmit(const void* ptr, PayloadLengthT len, int timeout)
{
    if(!pimpl->write(ptr, len, std::chrono::milliseconds(timeout)))
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::transmitStart(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK; 
}

llc::Error llc::transport::lowlevel::USB::transmitReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::transmitComplete(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::receive(void* ptr, PayloadLengthT len, int timeout)
{
    if(!pimpl->read(ptr, len, std::chrono::milliseconds(timeout)))
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::receiveStart(PayloadLengthT& len, ChannelIDT& chan_id, NodeIDT& node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::receiveReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::receiveComplete(int timeout)
{
    return Error::OK;
}
