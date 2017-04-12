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
#include <string>
#include <chrono>
#include <stdexcept>
#include <cstring>

#include <misccpp/drivers/SerialPort.hpp>

#include <misccpp/lowlevelcom/transport/transport_serial_port.hpp>
#include <misccpp/lowlevelcom/cobs.hpp>

llc::transport::lowlevel::SerialPort::SerialPort(const std::string& a_ttydev, int a_baudrate, bool ause_cobs, std::size_t acobs_buf_size) : 
    pimpl(new drivers::SerialPort()), 
    use_cobs(ause_cobs), 
    cobs_buf_size(acobs_buf_size),
    bytes_received(0), read_idx(0)
{
    pimpl->open(a_ttydev, a_baudrate);
    if(use_cobs)
    {
        txbuf = std::unique_ptr<uint8_t[]>(new uint8_t[cobs_buf_size]);
        rxbuf = std::unique_ptr<uint8_t[]>(new uint8_t[cobs_buf_size]);
    }
}

llc::transport::lowlevel::SerialPort::~SerialPort()
{
    
}
    
llc::Error llc::transport::lowlevel::SerialPort::transmit(const void* ptr, std::size_t len, int timeout)
{
    if(use_cobs)
    {
        const std::size_t outbuf_size = llc::COBS::calculateNewSize(len) + 2;
        
        if(outbuf_size > cobs_buf_size)
        {
            throw std::runtime_error("Too small buffers");
        }

        txbuf.get()[0] = llc::COBS::Delimiter;
        std::size_t processed = llc::COBS::encode(ptr, len, &txbuf.get()[1]);
        txbuf.get()[processed+1] = llc::COBS::Delimiter;
        
        if(!pimpl->write(txbuf.get(), processed + 2, std::chrono::milliseconds(timeout)))
        {
            return Error::Timeout;
        }
        else
        {
            return Error::OK;
        }
    }
    else
    {
        if(!pimpl->write(ptr, len, std::chrono::milliseconds(timeout)))
        {
            return Error::Timeout;
        }
        else
        {
            return Error::OK;
        }
    }
}

llc::Error llc::transport::lowlevel::SerialPort::transmitStart(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SerialPort::transmitReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SerialPort::transmitComplete(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SerialPort::receive(void* ptr, std::size_t len, int timeout)
{
    if(!use_cobs)
    {
        if(!pimpl->read(ptr, len, std::chrono::milliseconds(timeout)))
        {
            return Error::Timeout;
        }
    }
    else
    {
        if(read_idx + len > bytes_received)
        {
            throw std::runtime_error("Out of bounds");
        }
        
        memcpy(ptr, &(rxbuf.get()[read_idx]), len);
        read_idx += len;
    }
    
    return Error::OK;
}

// NOTE because of the RawIO design, if COBS then we receive the whole packet here to be able
// to decode before multiple receive calls (for PayloadLengthT, ChannelIDT, NodeIDT and CRCT optionally).
llc::Error llc::transport::lowlevel::SerialPort::receiveStart(int timeout)
{
    bytes_received = 0;
    read_idx = 0;
    
    if(use_cobs)
    {
        while(1)
        {
            uint8_t byte = 0;
            
            if(!pimpl->read(&byte, 1, std::chrono::milliseconds(timeout)))
            {
                return Error::Timeout;
            }
            
            if(bytes_received == 0)
            {
                if(byte == llc::COBS::Delimiter) // start of packet
                {
                    ++bytes_received;
                }
            }
            else
            {
                if(byte != llc::COBS::Delimiter)
                {
                    if(bytes_received + 1 > cobs_buf_size)
                    {
                        bytes_received = 0;
                        read_idx = 0;
                        throw std::runtime_error("Too small buffers");
                    }
                    
                    rxbuf.get()[bytes_received-1] = byte;
                    ++bytes_received;
                }
                else // end of packet
                {
                    --bytes_received;
                    
                    std::size_t new_count = llc::COBS::decode(rxbuf.get(), bytes_received, rxbuf.get());
                    
                    read_idx = 0;
                    bytes_received = new_count;
                    
                    break;
                }
            }
        }
    }
    
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SerialPort::receiveReset()
{
    bytes_received = 0;
    read_idx = 0;
    return Error::OK;
}

llc::Error llc::transport::lowlevel::SerialPort::receiveComplete(int timeout)
{
    bytes_received = 0;
    read_idx = 0;
    return Error::OK;
}
