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
 * Simple C++ SocketCAN interface.
 * ****************************************************************************
 */

#include <misccpp/drivers/SocketCAN.hpp>

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

static inline timespec timespec_from_ms(const uint32_t timeout_ms)
{
    timespec time;
    time.tv_sec = timeout_ms / 1e3;
    time.tv_nsec = (timeout_ms - (time.tv_sec * 1e3)) * 1e6;
    return time;
}

drivers::SocketCAN::SocketCAN() : sock(-1), use_fd(false)
{

}

drivers::SocketCAN::~SocketCAN()
{

}

bool drivers::SocketCAN::open(const char* port, bool ufd)
{
    use_fd = ufd;
    
    struct ifreq ifr;
    struct sockaddr_can addr;
    
    sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock < 0) { return false; }
    
    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);
    
    if(::ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        ::close(sock);
        sock = -1;
        return false;
    }
    
    addr.can_ifindex = ifr.ifr_ifindex;
    ::fcntl(sock, F_SETFL, O_NONBLOCK);
    
    if(::bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        ::close(sock);
        sock = -1;
        return false;
    }
    
    return true;
}

bool drivers::SocketCAN::isOpen()
{
    return sock != -1;
}

void drivers::SocketCAN::close()
{
    if(isOpen())
    {
        ::close(sock);
        sock = -1;
    }
}

bool drivers::SocketCAN::readImpl(uint32_t& cid, void* dst, std::size_t& count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    fd_set readsock;
    FD_ZERO(&readsock);
    FD_SET(sock, &readsock);
    struct timespec timeout_ts(timespec_from_ms(timeout_ms));
        
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
                    return false;
                }
                else
                {
                    cid = cf.can_id;
                    count = cf.can_dlc;
                    memcpy(dst, cf.data, cf.can_dlc);
                }
            }
            else
            {
                struct canfd_frame cf;
                
                recvbytes = ::read(sock, &cf, sizeof(struct canfd_frame));
                
                if(recvbytes != sizeof(struct canfd_frame))
                {
                    return false;
                }
                else
                {
                    cid = cf.can_id;
                    count = cf.len;
                    memcpy(dst, cf.data, cf.len);
                }
            }
        }
    }
    else if(rc == 0) // timeout
    {
        return false;
    }
    else // interrupt / other error
    {
        return false;
    }
     
    return true;
}

bool drivers::SocketCAN::writeImpl(const uint32_t& cid, const void* src, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    ssize_t retval = -1;
    
    // TODO FIXME what about EFF/RTR/ERR flags?
    
    if(!use_fd)
    {
        if(count >= CAN_MAX_DLEN) // too much
        {
            return false;
        }
        
        struct can_frame cf;
        cf.can_id = cid;
        cf.can_dlc = count;
        memcpy(cf.data, src, count);
        
        retval = ::write(sock, &cf, sizeof(struct can_frame));
        
        if(retval != sizeof(struct can_frame))
        {
            return false;
        }
    }
    else
    {
        if(count >= CANFD_MAX_DLEN) // too much
        {
            return false;
        }
        
        struct canfd_frame cf;
        cf.can_id = cid;
        cf.len = count;
        memcpy(cf.data, src, count);
        
        retval = ::write(sock, &cf, sizeof(struct canfd_frame));
        
        if(retval != sizeof(struct canfd_frame))
        {
            return false;
        }
    }
    
    return true;
}
