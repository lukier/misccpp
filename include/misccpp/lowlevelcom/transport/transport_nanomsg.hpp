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
 * NanoMSG transport.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_TRANSPORT_NANOMSG_HPP
#define LOWLEVELCOM_TRANSPORT_NANOMSG_HPP

#include <array>
#include <tuple>

#include <misccpp/lowlevelcom/utils.hpp>

#include <misccpp/nanomsg.hpp>

namespace llc
{
    
namespace transport
{

template<typename ... NMSGIOS>
class NanoMSGIOPoller;

template<bool MultiChannel = false, bool MultiNode = false>
class NanoMSGIO
{
public:
    static constexpr bool NeedsKnownInputMessage = false;
    static constexpr bool IsMultiChannel = MultiChannel;
    static constexpr bool IsMultiNode = MultiNode;
    
    class message_t
    {
    public:
        static constexpr std::size_t PrefixRequired = internal::ChannelIDSize<IsMultiChannel>::size + internal::NodeIDSize<IsMultiNode>::size;
        
        // default constructor
        explicit message_t() { }
        
        // allocation constructor
        explicit message_t(std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : msg(PrefixRequired + l)
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
        
        inline void swap(message_t& m) noexcept
        {
            std::swap(msg, m.msg);
        }
        
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
        friend class NanoMSGIO; // for raw access
        
        inline const nn::message_t& raw_message() const { return msg; }
        inline nn::message_t& raw_message() { return msg; }
        
        inline const void* getBufferAt(std::size_t offset) const { return static_cast<const void*>(&(static_cast<uint8_t*>(msg.data())[offset])); }
        inline void* getBufferAt(std::size_t offset) { return static_cast<void*>(&(static_cast<uint8_t*>(msg.data())[offset])); }
        
        mutable nn::message_t msg;
    };
    
    NanoMSGIO() = delete;
    
    NanoMSGIO(nn::socket_t& s) : socket(s) { }
    virtual ~NanoMSGIO() { }

    // non copyable
    NanoMSGIO(const NanoMSGIO&) = delete;
    NanoMSGIO& operator=(NanoMSGIO const&) = delete; 
    
    // move constructor
    inline NanoMSGIO(NanoMSGIO&& m) noexcept : socket(std::move(m.socket)) {  }
    
    // move assignment
    inline NanoMSGIO& operator=(NanoMSGIO&& m) noexcept { socket = std::move(m.socket); return *this; }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    Error receive(message_t& msg_out, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        const int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        
        socket.setsockopt<int>(NN_SOL_SOCKET, NN_RCVTIMEO, timeout_ms == 0 ? -1 : timeout_ms);
        socket.receive(msg_out.raw_message());
        return Error::OK;
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    Error transmit(const message_t& msg, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        const int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        
        socket.setsockopt<int>(NN_SOL_SOCKET, NN_SNDTIMEO, timeout_ms == 0 ? -1 : timeout_ms);
        socket.send(msg.raw_message());
        return Error::OK;
    }
    
    void lock() { }
    void unlock() { }

    template<typename ... NMSGIOS>
    using Poller = NanoMSGIOPoller<NMSGIOS...>;
private:
    template<typename ... NMSGIOS> friend class NanoMSGIOPoller;
    
    nn::socket_t& socket;
};

template<typename ... NMSGIOS>
class NanoMSGIOPoller
{
    static constexpr std::size_t IOSCount = sizeof...(NMSGIOS);
public:
    template<typename ... EventArgs>
    NanoMSGIOPoller(NMSGIOS& ... ios, EventArgs ... event)
    {
        static_assert(IOSCount == sizeof...(EventArgs), "Size mismatch");
        // fill array
        unsigned int cnt = 0;
        int x[] = { ( ios.socket.setFD(fds[cnt]), fds[cnt].events = event, cnt++ , 0)... };
        (void)x;
    }
    
    Error poll(int timeout = 0)
    {
        nn::poll(fds.data(), (int)IOSCount, timeout == 0 ? -1 : timeout);
        return Error::OK;
    }
private:
    std::array<nn_pollfd,IOSCount> fds;
};

}

}

#endif // LOWLEVELCOM_TRANSPORT_NANOMSG_HPP
