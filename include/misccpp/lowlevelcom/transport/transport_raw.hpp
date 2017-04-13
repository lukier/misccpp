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
 * Interface to create packet transports from low level transports.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOMM_TRANSPORT_RAW_HPP
#define LOWLEVELCOMM_TRANSPORT_RAW_HPP

#include <cassert>
#include <algorithm>
#include <type_traits>
#include <memory>

#include <misccpp/lowlevelcom/utils.hpp>

namespace llc
{
    
namespace transport
{
    
/**
 * RawIO creates Transport from Low Level IO. Makes packets with PacketLengthT and optionally NodeIDT and ChannelIDT.
 * If drop framing then PayloadLengthT, ChannelIDT and NodeIDT header and CRC footer are dropped.
 * NOTE: Receive may need allocated message of known size beforehand, depending on the low level transport! For now that's incompatible with Bridge/Mux
 */
template<typename SERIAL_IO, bool DropFraming = false, bool MultiChannel = false, bool MultiNode = false, typename CRC_PROVIDER = internal::NoCRC, typename ALLOCATOR_T = internal::DefaultAllocator>
class RawIO
{
public:    
    static constexpr bool NeedsKnownInputMessage = (DropFraming && !SERIAL_IO::ProvidesFieldsUpdated);
    static constexpr bool IsMultiChannel = MultiChannel;
    static constexpr bool IsMultiNode = MultiNode;
    static constexpr bool ToDropFraming = DropFraming;
    
    static_assert(!(ToDropFraming && !SERIAL_IO::SupportsFraming), "Low level doesn't support framing, switch to DropFraming");
    static_assert(!(SERIAL_IO::RequiresNodeID && (!IsMultiNode)), "Low level needs NodeID");
    static_assert(!(SERIAL_IO::RequiresChannelID && (!IsMultiChannel)), "Low level needs ChannelID");
    
    class message_t
    {
    public:
        typedef typename CRC_PROVIDER::crc_type crc_type;
        
        /**
         * Message will still contain PayloadLengthT, ChannelIDT and NodeIDT in the buffer even if DropFraming.
         * transmit/receive functions are responsible for managing the differences.
         */
        static constexpr std::size_t PrefixRequired = internal::ChannelIDSize<IsMultiChannel>::size + 
                                                      internal::NodeIDSize<IsMultiNode>::size + 
                                                      internal::PayloadLengthSize<true>::size +
                                                      CRC_PROVIDER::crc_suffix_size;
        
        // default constructor
        message_t() : buf(nullptr, &message_t::nodelete), len(0) { setChannelID(0); setNodeID(0); }
        
        // allocation constructor
        explicit message_t(std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : buf(static_cast<uint8_t*>(ALLOCATOR_T::malloc(PrefixRequired + l)), &ALLOCATOR_T::free), len(PrefixRequired + l) 
        { 
            if(buf.get() != 0)
            {
                updateSize(l);
                setChannelID(cid);
                setNodeID(nid);
            }
        }
        
        // external memory constructor
        explicit message_t(void* data, std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : buf(static_cast<uint8_t*>(data), &message_t::nodelete), len(PrefixRequired + l)
        { 
            if(buf.get() != 0)
            {
                updateSize(l);
                setChannelID(cid);
                setNodeID(nid);
            }
        }
        
        // move constructor
        inline message_t(message_t&& m) noexcept : buf(m.buf), len(m.len) { m.buf = 0; m.len = 0; }
        
        // move assignment
        inline message_t& operator=(message_t&& m) noexcept { m.swap(*this); return *this; }
        
        // non-copyable
        message_t(const message_t&) = delete;
        message_t& operator=(const message_t&) = delete;
        
        // destructor
        inline ~message_t() { len = 0; }
        
        // access
        inline const void* data() const { if(buf.get() != 0) { return static_cast<const void*>(getBufferAt(PrefixRequired)); } else { return nullptr; } }
        inline void* data() { if(buf.get() != 0) { return static_cast<void*>(getBufferAt(PrefixRequired)); } else { return nullptr; } }
        inline std::size_t size() const { return len; }
        
        // has data?
        inline explicit operator bool () const noexcept { return buf.get() != 0; }
        
        inline void swap(message_t& m) noexcept
        {
            std::swap(buf, m.buf);
            std::swap(len, m.len);
        }
        
        static inline constexpr bool supportsChannelID() { return MultiChannel; }
        // optional ChannelID get
        ChannelIDT getChannelID() const { if(MultiChannel) { return *static_cast<const ChannelIDT*>(getBufferAt(0)); } else { return 0; } }
        // optional ChannelID set
        inline void setChannelID(ChannelIDT n) { if(MultiChannel && (buf.get() != 0)) { *static_cast<ChannelIDT*>(getBufferAt(0)) = n; } }
        
        static inline constexpr bool supportsNodeID() { return MultiNode; }
        // optional NodeID get
        NodeIDT getNodeID() const { if(MultiNode) { return *static_cast<const NodeIDT*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size)); } else { return 0; } }
        // optional NodeID set
        inline void setNodeID(NodeIDT n) { if(MultiNode && (buf.get() != 0)) { *static_cast<NodeIDT*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size)) = n; } }

    private:
        friend class RawIO; // for CRC and raw access
        
        inline void* raw_data() const { return buf.get(); }
        inline const void* raw_data() { return buf.get(); }
        inline std::size_t raw_size() const { return PrefixRequired + len; }
        
        static inline constexpr bool supportsCRC() { return CRC_PROVIDER::crc_enabled; }
        
        // optional CRC compute (doesn't include framing header data)
        crc_type calculateCRC() const
        {
            if(CRC_PROVIDER::crc_enabled) { return CRC_PROVIDER::calculateCRC(len, getBufferAt(PrefixRequired)); } else { return 0; }
        }
        
        // optional read message CRC
        crc_type getCRC() const
        {
            if(CRC_PROVIDER::crc_enabled)
            {
                return *static_cast<crc_type*>(getBufferAt(PrefixRequired + len));
            }
            else
            {
                return 0;
            }
        }
        
        // optional write message CRC
        void writeCRC(crc_type crc) const
        {
            if(CRC_PROVIDER::crc_enabled)
            {
                *static_cast<crc_type*>(const_cast<void*>(getBufferAt(PrefixRequired + len))) = crc;
            }
        }
        
        inline const void* getBufferAt(std::size_t offset) const { return static_cast<const void*>(&(buf.get()[offset])); }
        inline void* getBufferAt(std::size_t offset) { return static_cast<void*>(&(buf.get()[offset])); }
        
        void updateSize(PayloadLengthT l) const
        {
            *static_cast<PayloadLengthT*>(const_cast<void*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size + internal::NodeIDSize<MultiNode>::size))) = l;
        }
        
        static inline void nodelete(void*) { }
        
        mutable std::unique_ptr<uint8_t,void(*)(void*)> buf;
        std::size_t len;
    };
    
    RawIO() = delete;
    
    RawIO(SERIAL_IO& asio) : sio(asio)
    {
        
    }
    
    virtual ~RawIO()
    {
        
    }
    
    // non copyable
    RawIO(const RawIO&) = delete;
    RawIO& operator=(RawIO const&) = delete; 
    
    // move constructor
    inline RawIO(RawIO&& m) noexcept : sio(m.sio) {  }
    
    // move assignment
    inline RawIO& operator=(RawIO&& m) noexcept { m.swap(*this); return *this; }
    
    inline void swap(RawIO& m) noexcept
    {
        std::swap(sio, m.sio);
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    Error receive(message_t& msg_out, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        const int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        
        Error rc = Error::OK;
        
        // NOTE: If ToDropFraming then it is assumed that msg_out contains
        // necessary data or receiveStart returns FieldsUpdated
        PayloadLengthT len = msg_out.size();
        NodeIDT node_id = msg_out.getNodeID();
        ChannelIDT chan_id = msg_out.getChannelID();
        
        // may be needed sometimes to prepare lowlevel backend
        rc = sio.receiveStart(len, chan_id, node_id, timeout_ms);
        if(rc != Error::OK) 
        { 
            if(rc == Error::FieldsUpdated)
            {
                // low level backend provided new info
                msg_out = message_t(len, chan_id, node_id);
            }
            else
            {
                sio.receiveReset(); 
                return LLC_HANDLE_ERROR(rc); 
            }
        }

        // If not dropping framing let's receive the header
        if(!ToDropFraming)
        {
            std::size_t read_pos = 0;
            
            uint8_t header_buf[message_t::PrefixRequired];
            
            // ask for header bytes
            rc = sio.receive(header_buf, message_t::PrefixRequired, timeout_ms);
            if(rc != Error::OK) 
            { 
                sio.receiveReset(); 
                return LLC_HANDLE_ERROR(rc); 
            }

            // read ChannelID
            if(IsMultiChannel)
            {
                const ChannelIDT* ptr = (ChannelIDT*)&header_buf[read_pos];
                chan_id = *ptr;
                read_pos += sizeof(ChannelIDT);
            }
            
            // read NodeID
            if(IsMultiNode)
            {
                const NodeIDT* ptr = (NodeIDT*)&header_buf[read_pos];
                node_id = *ptr;
                read_pos += sizeof(NodeIDT);
            }
            
            // read length
            const PayloadLengthT* lenptr = (PayloadLengthT*)&header_buf[read_pos];
            len = *lenptr;
            read_pos += sizeof(PayloadLengthT);
            
            // create message, as it isn't predefined
            msg_out = message_t(len, chan_id, node_id);
        }

        // If drop framing then the input message has to be allocated and of known length (+other header fields)
        // or return FieldsUpdated in receiveStart
        if(msg_out.data() == 0) 
        { 
            sio.receiveReset(); 
            return LLC_HANDLE_ERROR(Error::OutOfMemory); 
        }
               
        // receive payload
        rc = sio.receive(msg_out.data(), msg_out.size(), timeout_ms);
        if(rc != Error::OK) 
        { 
            sio.receiveReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
      
        // receive and verify CRC
        if(!ToDropFraming && message_t::supportsCRC())
        {
            typedef typename message_t::crc_type crc_type;
            // receive CRC
            crc_type data_crc = 0;
            rc = sio.receive(&data_crc, sizeof(crc_type), timeout_ms);
            if(rc != Error::OK) 
            { 
                sio.receiveReset(); 
                return LLC_HANDLE_ERROR(rc); 
            }
          
            // calculate CRC for the payload
            crc_type should_crc = msg_out.calculateCRC();
            
            // verify
            if(data_crc != should_crc) 
            { 
                sio.receiveReset(); 
                return LLC_HANDLE_ERROR(Error::CRCMismatch); 
            }
            
            // update CRC in the message
            msg_out.writeCRC(should_crc);
        }
        
        // may be needed sometimes to complete
        rc = sio.receiveComplete(timeout_ms);
        if(rc != Error::OK) 
        { 
            sio.receiveReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
            
        return rc;
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    Error transmit(const message_t& msg, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        const int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        
        Error rc = Error::OK;
        
        // may be needed sometimes to prepare the low level backend
        rc = sio.transmitStart(msg.getChannelID(), msg.getNodeID(), timeout_ms);
        if(rc != Error::OK) 
        { 
            sio.transmitReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
        
        // prepare CNC
        if(message_t::supportsCRC())
        {
            msg.writeCRC(msg.calculateCRC());
        }

        if(!ToDropFraming)
        {
            // transmit everything
            rc = sio.transmit(msg.raw_data(), msg.raw_size(), timeout_ms);
        }
        else
        {
            // transmit just payload
            rc = sio.transmit(msg.data(), msg.size(), timeout_ms);
        }
        
        if(rc != Error::OK) 
        { 
            sio.transmitReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
        
        // may be needed sometimes to complete
        rc = sio.transmitComplete(msg.getChannelID(), msg.getNodeID(), timeout_ms);
        if(rc != Error::OK) 
        { 
            sio.transmitReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
        
        return rc;
    }
    
    void lock() { sio.lock(); }
    void unlock() { sio.unlock(); }
private:
    SERIAL_IO& sio;
};

}

}

#endif // LOWLEVELCOMM_TRANSPORT_RAW_HPP
