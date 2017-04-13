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
 * Linux SPI based low level and high level packet transport.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_TRANSPORT_SPI_HPP
#define LOWLEVELCOM_TRANSPORT_SPI_HPP

#include <mutex>
#include <memory>

#include <misccpp/lowlevelcom/utils.hpp>

namespace drivers
{
class DevSPI;
}

namespace llc
{

namespace transport
{
    
namespace lowlevel
{

/**
 * SPI Dev Low Level Interface for RawIO.
 * NOTE: Fully compatible with RawIO framing and header/footer reads.
 */
class SPI
{
public:
    static constexpr bool ProvidesFieldsUpdated = false;
    static constexpr bool RequiresChannelID = false;
    static constexpr bool RequiresNodeID = false; 
    static constexpr bool SupportsFraming = true;
    
    SPI() = delete;
    SPI(const char* dev_path, uint32_t freq = 1000000, uint8_t bpw = 8, bool cpha = true, bool cpol = true, bool lsb_first = true);
    virtual ~SPI();
    
    // non copyable
    SPI(const SPI&) = delete;
    SPI& operator=(SPI const&) = delete; 
    
    // TODO FIXME move semantics
    
    // TX
    Error transmit(const void* ptr, PayloadLengthT len, int timeout);
    Error transmitStart(ChannelIDT chan_id, NodeIDT node_id, int timeout);
    Error transmitReset();
    Error transmitComplete(ChannelIDT chan_id, NodeIDT node_id, int timeout);
    
    // RX
    Error receive(void* ptr, PayloadLengthT, int timeout);
    Error receiveStart(PayloadLengthT& len, ChannelIDT& chan_id, NodeIDT& node_id, int timeout);
    Error receiveReset();
    Error receiveComplete(int timeout);
    
    void lock() { safety.lock(); }
    void unlock() { safety.unlock(); }
private:
    std::unique_ptr<drivers::DevSPI> pimpl;
    std::mutex safety;
};

}

/**
 * SPI Transport.
 * Byte data contains optionally NodeID & ChannelID. 
 * NOTE: Receive in SPI needs allocated message of known size beforehand! For now that's incompatible with Bridge/Mux
 */
template<bool MultiChannel = false, bool MultiNode = false>
class SPIIO
{
public:
    static constexpr bool NeedsKnownInputMessage = true;
    static constexpr bool IsMultiChannel = MultiChannel;
    static constexpr bool IsMultiNode = MultiNode;
    
    class message_t
    {
    public:
        static constexpr std::size_t PrefixRequired = internal::ChannelIDSize<MultiChannel>::size + internal::NodeIDSize<MultiNode>::size;
        
        // default constructor
        explicit message_t() : len(0), buf(nullptr, &message_t::nodelete) { }
        
        // allocation constructor
        explicit message_t(std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : len(PrefixRequired + l), buf(static_cast<uint8_t*>(malloc(PrefixRequired + l)), &message_t::deleter)
        { 
            setChannelID(cid);
            setNodeID(nid);
        }
        
        explicit message_t(void* data, std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : len(PrefixRequired + l), buf(static_cast<uint8_t*>(data), &message_t::nodelete)
        { 
            setChannelID(cid);
            setNodeID(nid);
        }
        
        // move constructor
        inline message_t(message_t&& m) noexcept : len(m.len), buf(std::move(m.buf)) {  }
        
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
        
        inline void swap(message_t& m) noexcept { std::swap(buf, m.buf); std::swap(len, m.len); }
        
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
        inline const void* getBufferAt(std::size_t offset) const { return static_cast<const void*>(&(buf.get()[offset])); }
        inline void* getBufferAt(std::size_t offset) { return static_cast<void*>(&(buf.get()[offset])); }
        
        static inline void nodelete(uint8_t*) { }
        static inline void deleter(uint8_t* ptr) { ::free(ptr); }
        
        friend class SPIIO; // for raw access

        std::size_t len;
        std::unique_ptr<uint8_t,void(*)(uint8_t*)> buf;
    };
    
    SPIIO() = delete;
    
    SPIIO(const char* dev_path, uint32_t freq = 1000000, uint8_t bpw = 8, bool cpha = true, bool cpol = true, bool lsb_first = true);
    virtual ~SPIIO();

    // non copyable
    SPIIO(const SPIIO&) = delete;
    SPIIO& operator=(SPIIO const&) = delete; 
    
    // move constructor
    inline SPIIO(SPIIO&& m) noexcept : pimpl(std::move(m.pimpl)) {  }
    
    // move assignment
    inline SPIIO& operator=(SPIIO&& m) noexcept { pimpl = std::move(m.pimpl); return *this; }
    
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
    std::unique_ptr<drivers::DevSPI> pimpl;
    std::mutex safety;    
};

}

}

#endif // LOWLEVELCOM_TRANSPORT_SPI_HPP
