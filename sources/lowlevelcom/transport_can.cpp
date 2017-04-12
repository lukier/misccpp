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
 * Linux SocketCAN based low level transport.
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
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <misccpp/lowlevelcom/transport/transport_can.hpp>

static inline timespec timespec_from_ms(const uint32_t timeout_ms)
{
    timespec time;
    time.tv_sec = timeout_ms / 1e3;
    time.tv_nsec = (timeout_ms - (time.tv_sec * 1e3)) * 1e6;
    return time;
}

llc::transport::lowlevel::CAN::CAN(const char* port, bool ufd) : sock(-1), use_fd(ufd)
{
    struct ifreq ifr;
    struct sockaddr_can addr;
    
    sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock < 0) { return; }
    
    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);
    
    if(::ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        ::close(sock);
        sock = -1;
        return;
    }
    
    addr.can_ifindex = ifr.ifr_ifindex;
    ::fcntl(sock, F_SETFL, O_NONBLOCK);
    
    if(::bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        ::close(sock);
        sock = -1;
        return;
    }
}

llc::transport::lowlevel::CAN::~CAN()
{
    if(sock > 0)
    {
        ::close(sock);
        sock = -1;
    }
}

llc::Error llc::transport::lowlevel::CAN::transmit(const void* ptr, std::size_t len, int timeout)
{
    if(sock < 0) { return Error::HardwareError; }
    
    ssize_t retval = -1;
    
    // TODO FIXME what about incremental packet assembly out of CAN, like serial port?
    // TODO FIXME what about EFF/RTR/ERR flags?
    // need to redesign
    
    if(!use_fd)
    {
        if(len >= CAN_MAX_DLEN) // too much
        {
            return Error::Unsupported;
        }
        
        struct can_frame cf;
        cf.can_id = tx_node;
        cf.can_dlc = len;
        memcpy(cf.data, ptr, len);
        
        retval = ::write(sock, &cf, sizeof(struct can_frame));
        
        if(retval == sizeof(struct can_frame))
        {
            return Error::OK;
        }
        else
        {
            return Error::HardwareError;
        }
    }
    else
    {
        if(len >= CANFD_MAX_DLEN) // too much
        {
            return Error::Unsupported;
        }
        
        struct canfd_frame cf;
        cf.can_id = tx_node;
        cf.len = len;
        memcpy(cf.data, ptr, len);
        
        retval = ::write(sock, &cf, sizeof(struct canfd_frame));
        
        if(retval == sizeof(struct canfd_frame))
        {
            return Error::OK;
        }
        else
        {
            return Error::HardwareError;
        }
    }
}

llc::Error llc::transport::lowlevel::CAN::transmitStart(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    tx_node = node_id; // NodeID is CAN ID
    
    return Error::OK;
}

llc::Error llc::transport::lowlevel::CAN::transmitReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::CAN::transmitComplete(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::CAN::receive(void* ptr, std::size_t len, int timeout)
{
    uint8_t* buf_as_bytes = static_cast<uint8_t*>(ptr);
    
    fd_set readsock;
    FD_ZERO(&readsock);
    FD_SET(sock, &readsock);
    struct timespec timeout_ts(timespec_from_ms(timeout));
    
    // TODO FIXME this kind of reading is no good, RawIO expects multi-part reads for ChannelIDT and NodeIDT
    // need to redesign
    
    int rc = ::pselect((sock + 1), &readsock, NULL, NULL, &timeout_ts, NULL);
    if(rc > 0)
    {
        if(FD_ISSET(sock, &readsock))
        {
            ssize_t recvbytes = 0;
            if(!use_fd)
            {
                struct can_frame cf;
                
                recvbytes = ::read(sock, &cf, sizeof(struct can_frame));
                
                if(recvbytes != sizeof(struct can_frame))
                {
                    return Error::HardwareError;
                }
                else
                {
                    if(cf.can_dlc != len) 
                    {
                        return Error::HardwareError;
                    }
                    
                    rx_node = cf.can_id;
                    memcpy(buf_as_bytes, &rx_node, sizeof(NodeIDT));
                    memcpy(&buf_as_bytes[sizeof(NodeIDT)], cf.data, cf.can_dlc);
                }
            }
            else
            {
                struct canfd_frame cf;
                
                recvbytes = ::read(sock, &cf, sizeof(struct canfd_frame));
                
                if(recvbytes != sizeof(struct canfd_frame))
                {
                    return Error::HardwareError;
                }
                else
                {
                    if(cf.len != len) 
                    {
                        return Error::HardwareError;
                    }
                    
                    rx_node = cf.can_id;
                    memcpy(buf_as_bytes, &rx_node, sizeof(NodeIDT));
                    memcpy(&buf_as_bytes[sizeof(NodeIDT)], cf.data, cf.len);
                }
            }
        }
    }
    else if(rc == 0) // timeout
    {
        return Error::Timeout;
    }
    else
    {
        return Error::HardwareError;
    }
    
    return Error::OK;
}

llc::Error llc::transport::lowlevel::CAN::receiveStart(int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::CAN::receiveReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::CAN::receiveComplete(int timeout)
{
    return Error::OK;
}
