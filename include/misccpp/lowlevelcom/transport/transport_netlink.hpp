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
 * Linux Netlink based low level transport.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_TRANSPORT_NETLINK_HPP
#define LOWLEVELCOM_TRANSPORT_NETLINK_HPP

#include <misccpp/lowlevelcom/utils.hpp>

struct nl_sock;
struct nlmsghdr;

namespace llc
{
    
namespace transport
{

class Netlink
{
public:
    class message_t
    {
    public:
        // default constructor
        explicit message_t();
        
        // allocation constructor
        explicit message_t(std::size_t l, NodeIDT nid = 0);
        
        // move constructor
        inline message_t(message_t&& m) noexcept /*: msg(std::move(m.msg)) */ {  }
        
        // move assignment
        inline message_t& operator=(message_t&& m) noexcept { m.swap(*this); return *this; }
        
        // non-copyable
        message_t(const message_t&) = delete;
        message_t& operator=(const message_t&) = delete;
        
        // destructor
        ~message_t();
        
        // access
        //inline const void* data() const { return static_cast<void*>(getBufferAt(internal::NodeIDSize<MultiNode>::size)); }
        //inline void* data() { return static_cast<void*>(getBufferAt(internal::NodeIDSize<MultiNode>::size)); }
        //inline std::size_t size() const { return msg.size() - internal::NodeIDSize<MultiNode>::size; }
        
        // has data?
        //inline operator bool () const noexcept { return msg.data() != 0; }
        
        inline void swap(message_t& m) noexcept
        {
            //std::swap(msg, m.msg);
        }
        
        // optional NodeID get
        NodeIDT getNodeID() const { return 0; }
        // optional NodeID set
        inline void setNodeID(NodeIDT n) {  }
        
    private:
        mutable struct nlmsghdr* hdr;
    };
    
    Netlink() = delete;
    
    Netlink(struct nl_sock* s) : socket(s) { }
    virtual ~Netlink();
    
    // non copyable
    Netlink(const Netlink&) = delete;
    Netlink& operator=(Netlink const&) = delete; 
    
    // TODO FIXME move semantics
    
    Error receive(message_t& msg_out, int timeout = 0);
    Error transmit(const message_t& msg, int timeout = 0);
private:
    struct nl_sock* socket;
};

}

}

#endif // LOWLEVELCOM_TRANSPORT_NETLINK_HPP
