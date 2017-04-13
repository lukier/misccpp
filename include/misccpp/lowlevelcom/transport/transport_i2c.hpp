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
 * Linux i2c-dev based transport.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_TRANSPORT_I2C_HPP
#define LOWLEVELCOM_TRANSPORT_I2C_HPP

#include <array>
#include <tuple>
#include <mutex>
#include <memory>

#include <misccpp/lowlevelcom/utils.hpp>

namespace drivers
{
class DevI2C;
}

namespace llc
{
    
namespace transport
{

/**
 * I2C Packet Transport.
 * NodeID is I2C address.
 * Byte data contains optionally only ChannelID. 
 * NOTE: Receive in I2C needs allocated message of known size beforehand! For now that's incompatible with Bridge/Mux
 */
template<bool MultiChannel = false>
class I2CIO
{
public:
    static constexpr bool NeedsKnownInputMessage = true;
    static constexpr bool IsMultiChannel = MultiChannel;
    static constexpr bool IsMultiNode = true;
    
    class message_t
    {
    public:
        // default constructor
        explicit message_t() : i2caddr(0), len(0), buf(nullptr, &message_t::nodelete) { }
        
        // allocation constructor
        explicit message_t(std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : len(PrefixRequired + l), buf(static_cast<uint8_t*>(malloc(PrefixRequired + l)), &message_t::deleter)
        { 
            setChannelID(cid);
            setNodeID(nid);
        }
        
        // external memory constructor
        explicit message_t(void* data, std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : len(PrefixRequired + l), buf(static_cast<uint8_t*>(data), &message_t::nodelete)
        { 
            setChannelID(cid);
            setNodeID(nid);
        }
        
        static constexpr std::size_t PrefixRequired = internal::ChannelIDSize<IsMultiChannel>::size;
        
        // move constructor
        inline message_t(message_t&& m) noexcept : i2caddr(m.i2caddr), len(m.len), buf(std::move(m.buf)) {  }
        
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
        
        inline void swap(message_t& m) noexcept { std::swap(buf, m.buf); std::swap(i2caddr, m.i2caddr); std::swap(len, m.len); }
        
        static inline constexpr bool supportsChannelID() { return IsMultiChannel; }
        // optional ChannelID get
        ChannelIDT getChannelID() const { if(IsMultiChannel) { return *static_cast<const ChannelIDT*>(getBufferAt(0)); } else { return 0; } }
        // optional ChannelID set
        inline void setChannelID(ChannelIDT n) { if(IsMultiChannel) { *static_cast<ChannelIDT*>(getBufferAt(0)) = n; } }
        
        static inline constexpr bool supportsNodeID() { return IsMultiNode; }
        // optional NodeID get
        NodeIDT getNodeID() const { return i2caddr; }
        // optional NodeID set
        inline void setNodeID(NodeIDT n) { i2caddr = n; }
        
    private:
        inline const void* getBufferAt(std::size_t offset) const { return static_cast<const void*>(&(buf.get()[offset])); }
        inline void* getBufferAt(std::size_t offset) { return static_cast<void*>(&(buf.get()[offset])); }
        
        static inline void nodelete(uint8_t*) { }
        static inline void deleter(uint8_t* ptr) { ::free(ptr); }
        
        friend class I2CIO; // for raw access
        
        NodeIDT i2caddr;
        std::size_t len;
        std::unique_ptr<uint8_t,void(*)(uint8_t*)> buf;
    };
    
    I2CIO() = delete;
    
    I2CIO(const char* dev_path, bool addr10b = false);
    virtual ~I2CIO();

    // non copyable
    I2CIO(const I2CIO&) = delete;
    I2CIO& operator=(I2CIO const&) = delete; 
    
    // move constructor
    inline I2CIO(I2CIO&& m) noexcept : pimpl(std::move(m.pimpl)) {  }
    
    // move assignment
    inline I2CIO& operator=(I2CIO&& m) noexcept { pimpl = std::move(m.pimpl); return *this; }
    
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
    NodeIDT tx_i2c_addr;
    std::unique_ptr<drivers::DevI2C> pimpl;
    std::mutex safety;    
};

}

}

#endif // LOWLEVELCOM_TRANSPORT_I2C_HPP
