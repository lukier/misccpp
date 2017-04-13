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
 * Linux SocketCAN based transport.
 * ****************************************************************************
 */
#include <string>
#include <chrono>
#include <stdexcept>
#include <cstring>

#include <misccpp/drivers/SocketCAN.hpp>

#include <misccpp/lowlevelcom/transport/transport_can.hpp>

template<bool MultiChannel>
llc::transport::CANIO<MultiChannel>::CANIO(const char* port, bool usefd) : pimpl(new drivers::SocketCAN())
{
    pimpl->open(port, usefd);
}

template<bool MultiChannel>
llc::transport::CANIO<MultiChannel>::~CANIO()
{
    
}

template<bool MultiChannel>
llc::Error llc::transport::CANIO<MultiChannel>::receiveImpl(message_t& msg_out, const int timeout_ms)
{
    uint32_t got_cid = 0;
    std::size_t got_len = 0;
    
    if(!pimpl->read(got_cid, msg_out.buf.data(), got_len, std::chrono::milliseconds(timeout_ms)))
    {
        return Error::HardwareError;
    }
    
    msg_out.len = got_len - message_t::PrefixRequired;
    msg_out.setNodeID(got_cid);
    
    return Error::OK;
}

template<bool MultiChannel>
llc::Error llc::transport::CANIO<MultiChannel>::transmitImpl(const message_t& msg_out, const int timeout_ms)
{
    if(!pimpl->write(msg_out.getNodeID(), msg_out.buf.data(), message_t::PrefixRequired + msg_out.len, std::chrono::milliseconds(timeout_ms)))
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}


template class llc::transport::CANIO<true>;
template class llc::transport::CANIO<false>;
