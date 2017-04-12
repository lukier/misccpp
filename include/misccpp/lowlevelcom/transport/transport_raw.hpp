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

#include <misccpp/lowlevelcom/utils.hpp>

namespace llc
{
    
namespace transport
{
    
template<typename SERIAL_IO, typename ALLOCATOR_T,  bool MultiChannel = false, bool MultiNode = false, typename CRC_PROVIDER = internal::NoCRC>
class RawIO
{
public:    
    static constexpr AccessT Access = SERIAL_IO::Access;
    static constexpr bool IsMultiChannel = MultiChannel;
    static constexpr bool IsMultiNode = MultiNode;
    // TODO static_asserts on RequiresNodeID and RequiresChannelID
    
    class message_t
    {
    public:
        typedef typename CRC_PROVIDER::crc_type crc_type;
        
        // default constructor
        message_t() : buf(0), len(0) { setChannelID(0); setNodeID(0); }
        
        // allocation constructor
        explicit message_t(std::size_t l, ChannelIDT cid = 0, NodeIDT nid = 0) : buf(ALLOCATOR_T::malloc(
                                                                                            internal::ChannelIDSize<MultiChannel>::size + 
                                                                                            internal::NodeIDSize<MultiNode>::size + 
                                                                                            sizeof(PayloadLengthT) + 
                                                                                            l + CRC_PROVIDER::crc_suffix_size
                                                                                                        )
                                                                                 ), len(l) 
        { 
            assert(buf != nullptr);
            
            if(buf != 0)
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
        inline ~message_t() { if(buf != 0) { ALLOCATOR_T::free(buf); } len = 0; }
        
        // access
        inline const void* data() const { if(buf != 0) { return static_cast<void*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size + internal::NodeIDSize<MultiNode>::size + sizeof(PayloadLengthT))); } else { return buf; } }
        inline void* data() { if(buf != 0) { return static_cast<void*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size + internal::NodeIDSize<MultiNode>::size + sizeof(PayloadLengthT))); } else { return buf; } }
        inline std::size_t size() const { return len; }
        
        // has data?
        inline explicit operator bool () const noexcept { return buf != 0; }
        
        inline void swap(message_t& m) noexcept
        {
            std::swap(buf, m.buf);
            std::swap(len, m.len);
        }
        
        static inline constexpr bool supportsChannelID() { return MultiChannel; }
        // optional ChannelID get
        ChannelIDT getChannelID() const { if(MultiChannel) { return *static_cast<ChannelIDT*>(getBufferAt(0)); } else { return 0; } }
        // optional ChannelID set
        inline void setChannelID(ChannelIDT n) { if(MultiChannel && (buf != 0)) { *static_cast<ChannelIDT*>(getBufferAt(0)) = n; } }
        
        static inline constexpr bool supportsNodeID() { return MultiNode; }
        // optional NodeID get
        NodeIDT getNodeID() const { if(MultiNode) { return *static_cast<NodeIDT*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size)); } else { return 0; } }
        // optional NodeID set
        inline void setNodeID(NodeIDT n) { if(MultiNode && (buf != 0)) { *static_cast<NodeIDT*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size)) = n; } }

    private:
        friend class RawIO; // for CRC and raw access
        
        inline void* raw_data() const { return buf; }
        inline const void* raw_data() { return buf; }
        inline std::size_t raw_size() const { return internal::ChannelIDSize<MultiChannel>::size + internal::NodeIDSize<MultiNode>::size + sizeof(PayloadLengthT) + len + CRC_PROVIDER::crc_suffix_size; }
        
        static inline constexpr bool supportsCRC() { return CRC_PROVIDER::crc_enabled; }
        
        // optional CRC compute
        crc_type calculateCRC() const
        {
            if(CRC_PROVIDER::crc_enabled) { return CRC_PROVIDER::calculateCRC(len, getBufferAt(internal::ChannelIDSize<MultiChannel>::size + 
                                                                                               internal::NodeIDSize<MultiNode>::size + 
                                                                                               sizeof(PayloadLengthT))); } else { return 0; }
        }
        
        // optional read message CRC
        crc_type getCRC() const
        {
            if(CRC_PROVIDER::crc_enabled)
            {
                return *static_cast<crc_type*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size + 
                                               internal::NodeIDSize<MultiNode>::size + sizeof(PayloadLengthT) + len));
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
                *static_cast<crc_type*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size + internal::NodeIDSize<MultiNode>::size + sizeof(PayloadLengthT) + len)) = crc;
            }
        }
        
        inline void* getBufferAt(std::size_t offset) const  { return static_cast<void*>(&(static_cast<uint8_t*>(buf)[offset])); }
        
        void updateSize(PayloadLengthT l) const
        {
            *static_cast<PayloadLengthT*>(getBufferAt(internal::ChannelIDSize<MultiChannel>::size + internal::NodeIDSize<MultiNode>::size)) = l;
        }
        
        mutable void* buf;
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
    
    Error receive(message_t& msg_out, int timeout = 0)
    {
        Error rc = Error::OK;
        
        if((Access == AccessT::W)) { return Error::Unsupported; }
        
        // may be needed sometimes to prepare
        rc = sio.receiveStart(timeout);
        if(rc != Error::OK) 
        { 
            sio.receiveReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
        
        constexpr std::size_t min_bytes_to_read = internal::ChannelIDSize<MultiChannel>::size + 
                                                  internal::NodeIDSize<MultiNode>::size + 
                                                  sizeof(PayloadLengthT);
        
        uint8_t header_buf[min_bytes_to_read];
        std::size_t read_pos = 0;
        
        rc = sio.receive(header_buf, min_bytes_to_read, timeout);
        if(rc != Error::OK) 
        { 
            sio.receiveReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }

        ChannelIDT chan_id = 0;
        
        // read ChannelID
        if(IsMultiChannel)
        {
            const ChannelIDT* ptr = (ChannelIDT*)&header_buf[read_pos];
            chan_id = *ptr;
            read_pos += sizeof(ChannelIDT);
        }
        
        NodeIDT node_id = 0;
        
        // read NodeID
        if(IsMultiNode)
        {
            const NodeIDT* ptr = (NodeIDT*)&header_buf[read_pos];
            node_id = *ptr;
            read_pos += sizeof(NodeIDT);
        }
        
        // read length
        PayloadLengthT len = 0;
        
        const PayloadLengthT* lenptr = (PayloadLengthT*)&header_buf[read_pos];
        len = *lenptr;
        read_pos += sizeof(PayloadLengthT);
                
        // create message
        msg_out = message_t(len, chan_id, node_id);
        if(msg_out.data() == 0) 
        { 
            sio.receiveReset(); 
            return LLC_HANDLE_ERROR(Error::OutOfMemory); 
        }
               
        // receive payload
        rc = sio.receive(msg_out.data(), msg_out.size(), timeout);
        if(rc != Error::OK) 
        { 
            sio.receiveReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
      
        // receive and verify CRC
        if(message_t::supportsCRC())
        {
            typedef typename message_t::crc_type crc_type;
            // receive CRC
            crc_type data_crc = 0;
            rc = sio.receive(&data_crc, sizeof(crc_type), timeout);
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
        rc = sio.receiveComplete(timeout);
        if(rc != Error::OK) 
        { 
            sio.receiveReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
            
        return rc;
    }
    
    Error transmit(const message_t& msg, int timeout = 0)
    {
        Error rc = Error::OK;
        
        if((Access == AccessT::R)) { return Error::Unsupported; }
        
        // may be needed sometimes to prepare
        rc = sio.transmitStart(msg.getChannelID(), msg.getNodeID(), timeout);
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

        // transmit everything
        rc = sio.transmit(msg.raw_data(), msg.raw_size(), timeout);
        if(rc != Error::OK) 
        { 
            sio.transmitReset(); 
            return LLC_HANDLE_ERROR(rc); 
        }
        
        // may be needed sometimes to complete
        rc = sio.transmitComplete(msg.getChannelID(), msg.getNodeID(), timeout);
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
