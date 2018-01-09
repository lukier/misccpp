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
 * ZeroMQ Transport.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_TRANSPORT_ZEROMQ_HPP
#define LOWLEVELCOM_TRANSPORT_ZEROMQ_HPP

#include <array>
#include <tuple>
#include <functional>

#include <misccpp/zmq.hpp>

#include <misccpp/lowlevelcom/utils.hpp>

namespace llc
{
    
namespace transport
{

template<typename ... ZMQIOS>
class ZeroMQIOPoller;
    
template<bool MultiChannel = false, bool MultiNode = false>
class ZeroMQIO
{
public:
    static constexpr bool NeedsKnownInputMessage = false;
    static constexpr bool IsMultiChannel = MultiChannel;
    static constexpr bool IsMultiNode = MultiNode;
    
    class message_t
    {
    public:
        typedef std::function<void(void*)> DeleterT;
        static constexpr std::size_t PrefixRequired = internal::ChannelIDSize<MultiChannel>::size + internal::NodeIDSize<IsMultiNode>::size;
        
        // default constructor
        explicit message_t() { }
        
        // allocation constructor
        explicit message_t(std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : msg(PrefixRequired + l)
        { 
            setChannelID(cid);
            setNodeID(nid);
        }
        
        // zerocopy constructor
        explicit message_t(void* data, std::size_t l, DeleterT& cb, ChannelIDT cid = 0, NodeIDT nid = 0) : msg(data, PrefixRequired + l, &message_t::zerocopy_free_proxy, &cb)
        { 
            setChannelID(cid);
            setNodeID(nid);
        }
        
        // move constructor
        inline message_t(message_t&& m) noexcept : msg(std::move(m.msg)) {  }
        
        // move assignment
        inline message_t& operator=(message_t&& m) noexcept { m.swap(*this); return *this; }
        
        // non-copyable
        message_t(const message_t&) = delete;
        message_t& operator=(const message_t&) = delete;
        
        // destructor
        inline ~message_t() {  }
        
        // access
        inline const void* data() const { return static_cast<const void*>(getBufferAt(PrefixRequired)); }
        inline void* data() { return static_cast<void*>(getBufferAt(PrefixRequired)); }
        inline std::size_t size() const { return msg.size() - PrefixRequired; }
        
        // has data?
        inline operator bool () const noexcept { return msg.data() != 0; }
        
        inline void swap(message_t& m) noexcept { std::swap(msg, m.msg); }
        
        static inline constexpr bool supportsChannelID() { return IsMultiChannel; }
        // optional ChannelID get
        ChannelIDT getChannelID() const { if(IsMultiChannel) { return *static_cast<const ChannelIDT*>(getBufferAt(0)); } else { return 0; } }
        // optional ChannelID set
        inline void setChannelID(ChannelIDT n) { if(IsMultiChannel) { *static_cast<ChannelIDT*>(getBufferAt(0)) = n; } }
        
        static inline constexpr bool supportsNodeID() { return IsMultiNode; }
        // optional NodeID get
        NodeIDT getNodeID() const { if(IsMultiNode) { return *static_cast<const NodeIDT*>(getBufferAt(internal::ChannelIDSize<IsMultiChannel>::size)); } else { return 0; } }
        // optional NodeID set
        inline void setNodeID(NodeIDT n) { if(IsMultiNode) { *static_cast<NodeIDT*>(getBufferAt(internal::ChannelIDSize<IsMultiChannel>::size)) = n; } }
        
    private:
        friend class ZeroMQIO; // for raw access
        
        inline zmq::message_t& raw_message() const { return msg; }
        inline zmq::message_t& raw_message() { return msg; }
        
        static void zerocopy_free_proxy(void *data, void *hint)
        {
            DeleterT* fun = static_cast<DeleterT*>(hint);
            
            if(fun != nullptr)
            {
                if(*fun)
                {
                    (*fun)(data);
                }
            }
        }
        inline const void* getBufferAt(std::size_t offset) const { return static_cast<const void*>(&(static_cast<uint8_t*>(msg.data())[offset])); }
        inline void* getBufferAt(std::size_t offset) { return static_cast<void*>(&(static_cast<uint8_t*>(msg.data())[offset])); }
        
        mutable zmq::message_t msg;
    };
    
    ZeroMQIO() = delete;
    
    ZeroMQIO(zmq::socket_t& s) : socket(s) { }
    virtual ~ZeroMQIO() { }

    // non copyable
    ZeroMQIO(const ZeroMQIO&) = delete;
    ZeroMQIO& operator=(ZeroMQIO const&) = delete; 
    
    // move constructor
    inline ZeroMQIO(ZeroMQIO&& m) noexcept : socket(std::move(m.socket)) {  }
    
    // move assignment
    inline ZeroMQIO& operator=(ZeroMQIO&& m) noexcept { socket = std::move(m.socket); return *this; }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    Error receive(message_t& msg_out, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        
        if(timeout_ms == 0) { timeout_ms = -1;} 
        socket.setsockopt(ZMQ_RCVTIMEO, &timeout_ms, sizeof(int));
        socket.recv(&msg_out.raw_message());
        return Error::OK;
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    Error transmit(const message_t& msg, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        
        if(timeout_ms == 0) { timeout_ms = -1;} 
        socket.setsockopt(ZMQ_SNDTIMEO, &timeout_ms, sizeof(int));
        socket.send(msg.raw_message());
        return Error::OK;
    }
    
    void lock() { }
    void unlock() { }
    
    template<typename ... ZMQIOS>
    using Poller = ZeroMQIOPoller<ZMQIOS...>;
private:
    template<typename ... ZMQIOS> friend class ZeroMQIOPoller;
    
    zmq::socket_t& socket;
};

template<typename ... ZMQIOS>
class ZeroMQIOPoller
{
    static constexpr std::size_t IOSCount = sizeof...(ZMQIOS);
public:
    template<typename ... EventArgs>
    ZeroMQIOPoller(ZMQIOS& ... ios, EventArgs ... event)
    {
        static_assert(IOSCount == sizeof...(EventArgs), "Size mismatch");
        // fill array
        unsigned int cnt = 0;
        int x[] = { ( fds[cnt].socket = ios.socket.operator void*(), fds[cnt].events = event, cnt++ , 0)... };
        (void)x;
    }
    
    Error poll(int timeout = 0)
    {
        if(timeout == 0) { timeout = -1;} 
        zmq::poll(fds.data(), (int)IOSCount, timeout);
        return Error::OK;
    }
private:
    std::array<zmq::pollitem_t,IOSCount> fds;
};

}

}

#endif // LOWLEVELCOM_TRANSPORT_ZEROMQ_HPP
