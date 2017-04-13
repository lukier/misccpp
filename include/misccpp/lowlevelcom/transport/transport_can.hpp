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
#ifndef LOWLEVELCOM_TRANSPORT_CAN_HPP
#define LOWLEVELCOM_TRANSPORT_CAN_HPP

#include <array>
#include <tuple>
#include <mutex>
#include <memory>

#include <misccpp/lowlevelcom/utils.hpp>

namespace drivers
{
class SocketCAN;
}

namespace llc
{
    
namespace transport
{

/**
 * CAN Transport.
 * NodeID is CAN ID.
 * Byte data contains optionally only ChannelID. 
 */
template<bool MultiChannel = false>
class CANIO
{
public:
    static constexpr bool NeedsKnownInputMessage = false;
    static constexpr bool IsMultiChannel = MultiChannel;
    static constexpr bool IsMultiNode = true;
    
    class message_t
    {
    public:
        static constexpr std::size_t CANMaximumPacketSize = 64;
        static constexpr std::size_t PrefixRequired = internal::ChannelIDSize<IsMultiChannel>::size;
        
        // default constructor
        explicit message_t() : canid(0), len(0) { }
        
        // allocation constructor
        explicit message_t(std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : canid(0), len(std::max(CANMaximumPacketSize,PrefixRequired + l))
        { 
            setChannelID(cid);
            setNodeID(nid);
        }
        
        // move constructor
        inline message_t(message_t&& m) noexcept : canid(m.canid), len(m.len), buf(std::move(m.buf)) {  }
        
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
        inline std::size_t size() const { return len - PrefixRequired; }
        
        // has data?
        inline operator bool () const noexcept { return len != 0; }
        
        inline void swap(message_t& m) noexcept { std::swap(buf, m.buf); std::swap(canid, m.canid); std::swap(len, m.len); }
        
        static inline constexpr bool supportsChannelID() { return IsMultiChannel; }
        // optional ChannelID get
        ChannelIDT getChannelID() const { if(IsMultiChannel) { return *static_cast<const ChannelIDT*>(getBufferAt(0)); } else { return 0; } }
        // optional ChannelID set
        inline void setChannelID(ChannelIDT n) { if(IsMultiChannel) { *static_cast<ChannelIDT*>(getBufferAt(0)) = n; } }
        
        static inline constexpr bool supportsNodeID() { return IsMultiNode; }
        // optional NodeID get
        NodeIDT getNodeID() const { return canid; }
        // optional NodeID set
        inline void setNodeID(NodeIDT n) { canid = n; }
        
    private:
        inline const void* getBufferAt(std::size_t offset) const { return static_cast<const void*>(&(buf[offset])); }
        inline void* getBufferAt(std::size_t offset) { return static_cast<void*>(&(buf[offset])); }
        
        friend class CANIO; // for raw access
        
        NodeIDT canid;
        std::size_t len;
        std::array<uint8_t, CANMaximumPacketSize> buf;
    };
    
    CANIO() = delete;
    
    CANIO(const char* port, bool usefd = false);
    virtual ~CANIO();

    // non copyable
    CANIO(const CANIO&) = delete;
    CANIO& operator=(CANIO const&) = delete; 
    
    // move constructor
    inline CANIO(CANIO&& m) noexcept : pimpl(std::move(m.pimpl)) {  }
    
    // move assignment
    inline CANIO& operator=(CANIO&& m) noexcept { pimpl = std::move(m.pimpl); return *this; }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    Error receive(message_t& msg_out, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        const int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        return receiveImpl(msg_out, timeout_ms);
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    Error transmit(const message_t& msg, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        const int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        return transmitImpl(msg, timeout_ms);
    }
    
    void lock() { safety.lock(); }
    void unlock() { safety.unlock(); }

private:
    Error receiveImpl(message_t& msg_out, const int timeout_ms);
    Error transmitImpl(const message_t& msg_out, const int timeout_ms);
    std::unique_ptr<drivers::SocketCAN> pimpl;
    std::mutex safety;    
};

}

}

#endif // LOWLEVELCOM_TRANSPORT_CAN_HPP
