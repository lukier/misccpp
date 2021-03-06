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
#ifndef LOWLEVELCOMM_TRANSPORT_USB_HPP
#define LOWLEVELCOMM_TRANSPORT_USB_HPP

#include <mutex>
#include <memory>

#include <misccpp/lowlevelcom/utils.hpp>

namespace drivers
{
class DevUSB;
}

namespace llc
{
    
namespace transport
{
    
namespace lowlevel
{

class USB
{
public:
    static constexpr bool ProvidesFieldsUpdated = false;
    static constexpr bool RequiresChannelID = false;
    static constexpr bool RequiresNodeID = false;
    static constexpr bool SupportsFraming = true;
    
    USB() = delete;
    USB(uint16_t a_vid, uint16_t a_pid, uint8_t a_ep_in = 0xFF, uint8_t a_ep_out = 0xFF);
    virtual ~USB();
    
    // non copyable
    USB(const USB&) = delete;
    USB& operator=(USB const&) = delete; 
    
    // TODO FIXME move semantics
    
    // TX
    Error transmit(const void* ptr, PayloadLengthT len, int timeout);
    Error transmitStart(ChannelIDT chan_id, NodeIDT node_id, int timeout);
    Error transmitReset();
    Error transmitComplete(ChannelIDT chan_id, NodeIDT node_id, int timeout);
    
    // RX
    Error receive(void* ptr, PayloadLengthT len, int timeout);
    Error receiveStart(PayloadLengthT& len, ChannelIDT& chan_id, NodeIDT& node_id, int timeout);
    Error receiveReset();
    Error receiveComplete(int timeout);
    
    void lock() { safety.lock(); }
    void unlock() { safety.unlock(); }
private:
    std::unique_ptr<drivers::DevUSB> pimpl;
    std::mutex safety;
};

}

}

}

#endif // LOWLEVELCOMM_TRANSPORT_USB_HPP
