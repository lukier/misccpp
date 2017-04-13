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
 * Linux Serial Port based low level transport.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_TRANSPORT_SERIAL_PORT_HPP
#define LOWLEVELCOM_TRANSPORT_SERIAL_PORT_HPP

#include <string>
#include <memory>
#include <mutex>

#include <misccpp/lowlevelcom/utils.hpp>

namespace drivers
{
class SerialPort;
}

namespace llc
{
    
namespace transport
{
    
namespace lowlevel
{
    
class SerialPort
{
public:
    static constexpr bool ProvidesFieldsUpdated = false;
    static constexpr bool RequiresChannelID = false;
    static constexpr bool RequiresNodeID = false; 
    static constexpr bool SupportsFraming = true;
    
    SerialPort() = delete;
    SerialPort(const char* a_ttydev, int a_baudrate = 115200, bool ause_cobs = false, std::size_t acobs_buf_size = 1024*1024);
    virtual ~SerialPort();
    
    // non copyable
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(SerialPort const&) = delete; 
    
    // TODO FIXME move semantics
    
    Error transmit(const void* ptr, PayloadLengthT len, int timeout);
    Error transmitStart(ChannelIDT chan_id, NodeIDT node_id, int timeout);
    Error transmitReset();
    Error transmitComplete(ChannelIDT chan_id, NodeIDT node_id, int timeout);
    
    Error receive(void* ptr, PayloadLengthT len, int timeout);
    Error receiveStart(PayloadLengthT& len, ChannelIDT& chan_id, NodeIDT& node_id, int timeout);
    Error receiveReset();
    Error receiveComplete(int timeout);
    
    void lock() { safety.lock(); }
    void unlock() { safety.unlock(); }
private:
    std::unique_ptr<drivers::SerialPort> pimpl;
    bool use_cobs;
    std::size_t cobs_buf_size;
    std::unique_ptr<uint8_t[]> txbuf, rxbuf;
    std::size_t bytes_received, read_idx;
    std::mutex safety;
};

}

}

}

#endif // LOWLEVELCOM_TRANSPORT_SERIAL_PORT_HPP
