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
 * Transport multiplexer - multiplex multiple transport into a single one.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_MUX_HPP
#define LOWLEVELCOM_MUX_HPP

#include <cstring>
#include <array>
#include <tuple>
#include <memory>
#include <utility>
#include <type_traits>

#include <misccpp/lowlevelcom/utils.hpp>

namespace llc
{
    
namespace internal
{
    template <bool... b> struct static_all_of;
    
    // do recursion if the first argument is true
    template <bool... tail>
    struct static_all_of<true, tail...> : static_all_of<tail...> {};
    
    // end recursion if first argument is false
    template <bool... tail>
    struct static_all_of<false, tail...> : std::false_type {};
    
    // end recursion if no more arguments need to be processed
    template <> struct static_all_of<> : std::true_type {};
    
    template<typename MASTER_MSG_T, typename SLAVEIOT>
    static Error slave_receive_final(SLAVEIOT& sio, ChannelIDT cid, MASTER_MSG_T& msg_out, int timeout)
    {
        typedef MASTER_MSG_T master_message_t;
        typedef typename SLAVEIOT::message_t slave_message_t;
        
        Error rc = Error::OK;
        
        // receive
        slave_message_t msg_in;
        rc = sio.receive(msg_in, timeout);
        if(rc != Error::OK) { return rc; }
        
        // potentially serious
        if(msg_in.getChannelID() != cid)
        {
            return Error::ChannelError;
        }
        
        // create master message
        msg_out = master_message_t(msg_in.size(), msg_in.getChannelID(), msg_in.getNodeID());
        if(msg_out.data() == 0) { return Error::OutOfMemory; }
        
        /// @note memcopy
        std::memcpy(msg_out.data(), msg_in.data(), msg_in.size());
        
        return rc;
    }
    
    template<typename MASTER_MSG_T, int index, typename... SLAVE_IOS>
    struct slave_receive 
    {
        Error operator()(std::size_t idx, ChannelIDT cid, MASTER_MSG_T& msg_out, int timeout, std::tuple<SLAVE_IOS...>& t) 
        {
            if(index == idx)
            {
                typedef typename std::remove_reference<typename std::tuple_element<index,std::tuple<SLAVE_IOS...>>::type>::type SlaveIOT;
                return slave_receive_final<MASTER_MSG_T, SlaveIOT>(std::get<index>(t), cid, msg_out, timeout);
            }
            else
            {
                // go further
                return slave_receive<MASTER_MSG_T, index - 1, SLAVE_IOS...>{}(idx, cid, msg_out, timeout, t);
            }
        }
    };
    
    template<typename MASTER_MSG_T, typename... SLAVE_IOS>
    struct slave_receive<MASTER_MSG_T, 0, SLAVE_IOS...> 
    {
        static constexpr int index = 0;
        
        Error operator()(std::size_t idx, ChannelIDT cid, MASTER_MSG_T& msg_out, int timeout, std::tuple<SLAVE_IOS...>& t) 
        {
            if(index == idx)
            {
                typedef typename std::remove_reference<typename std::tuple_element<index,std::tuple<SLAVE_IOS...>>::type>::type SlaveIOT;
                return slave_receive_final<MASTER_MSG_T, SlaveIOT>(std::get<index>(t), cid, msg_out, timeout);
            }
            else
            {
                return Error::Unsupported;
            }
        }
    };
    
    template<typename SLAVEIOT>
    static Error slave_transmit_final(SLAVEIOT& sio, ChannelIDT cid, NodeIDT nid, std::size_t len, const void* data, int timeout)
    {
        typedef typename SLAVEIOT::message_t slave_message_t;
        
        Error rc = Error::OK;
        
        // receive
        slave_message_t msg_in(len, cid, nid);
        if(msg_in.data() == 0) { return Error::OutOfMemory; }
        
        /// @note memcopy
        std::memcpy(msg_in.data(), data, len);
        
        // transmit
        rc = sio.transmit(msg_in, timeout);
        
        return rc;
    }
    
    template<int index, typename... SLAVE_IOS>
    struct slave_transmit 
    {
        Error operator()(std::size_t idx, ChannelIDT cid, NodeIDT nid, std::size_t len, const void* data, int timeout, std::tuple<SLAVE_IOS...>& t) 
        {
            if(index == idx)
            {
                typedef typename std::remove_reference<typename std::tuple_element<index,std::tuple<SLAVE_IOS...>>::type>::type SlaveIOT;
                return slave_transmit_final<SlaveIOT>(std::get<index>(t), cid, nid, len, data, timeout);
            }
            else
            {
                // go further
                return slave_transmit<index - 1, SLAVE_IOS...>{}(idx, cid, nid, len, data, timeout, t);
            }
        }
    };
    
    template<typename... SLAVE_IOS>
    struct slave_transmit<0, SLAVE_IOS...> 
    {
        static constexpr int index = 0;
        
        Error operator()(std::size_t idx, ChannelIDT cid, NodeIDT nid, std::size_t len, const void* data, int timeout, std::tuple<SLAVE_IOS...>& t) 
        {
            if(index == idx)
            {
                typedef typename std::remove_reference<typename std::tuple_element<index,std::tuple<SLAVE_IOS...>>::type>::type SlaveIOT;
                return slave_transmit_final<SlaveIOT>(std::get<index>(t), cid, nid, len, data, timeout);
            }
            else
            {
                return Error::Unsupported;
            }
        }
    };
}
    
template<typename IO_SLAVE, ChannelIDT SLAVE_CHANNEL>
struct SlaveChannelPair
{
    typedef IO_SLAVE SlaveT;
    static constexpr ChannelIDT SlaveChannel = SLAVE_CHANNEL;
};
    
template<typename IO_MASTER, typename ... IO_SLAVE_PAIRS>
class Multiplexer
{
public:    
    typedef typename IO_MASTER::message_t master_message_t;
    
    static constexpr std::size_t SlaveCount = sizeof...(IO_SLAVE_PAIRS);
    
    static_assert(IO_MASTER::message_t::supportsChannelID() == true, "Master Transceiver must support ChannelID");
    static_assert(internal::static_all_of<IO_SLAVE_PAIRS::SlaveT::message_t::supportsChannelID()...>::value, "Slave Transceivers must support ChannelID");
    
    Multiplexer() = delete;
    Multiplexer(IO_MASTER& iom, typename IO_SLAVE_PAIRS::SlaveT& ...ios) : master_io(iom), slave_ios(std::forward<typename IO_SLAVE_PAIRS::SlaveT&>(ios)...) 
    { 
        unsigned int cnt = 0;
        int x[] = { ( slave_channel_ids[cnt++] = IO_SLAVE_PAIRS::SlaveChannel , 0)... };
        (void)x;
    }
    
    virtual ~Multiplexer() 
    { 
        
    }
    
    // non copyable
    Multiplexer(const Multiplexer&) = delete;
    Multiplexer& operator=(Multiplexer const&) = delete; 
    
    // TODO FIXME move semantics
    
protected:
    Error forwardMasterToSlave(int timeout_ms = 0, int timeout_sl = 0)
    {
        Error rc = Error::OK;
        
        // receive with prefix
        master_message_t msg_in;
        rc = master_io.receive(msg_in, timeout_ms);
        if(rc != Error::OK) { return rc; }
        
        // delegate this ugly job
        rc = transmitToSlave(msg_in.getChannelID(), msg_in.getNodeID(), msg_in.size(), msg_in.data(), timeout_sl);
        if(rc != Error::OK) { return rc; }
        
        return rc;
    }
    
    Error forwardSlaveToMaster(std::size_t slave_idx, int timeout_ms = 0, int timeout_sl = 0)
    {
        Error rc = Error::OK;
        
        if(slave_idx >= SlaveCount)
        {
            return Error::Unsupported;
        }
        
        ChannelIDT cid = slave_channel_ids[slave_idx];
        master_message_t msg_out;
        
        // well this is difficult
        rc = receiveFromSlave<master_message_t>(slave_idx, cid, msg_out, timeout_sl);
        if(rc != Error::OK) { return rc; }

        // transmit to master
        rc = master_io.transmit(msg_out, timeout_ms);

        return rc;
    }

private:
    template<typename... SLAVE_IOS>
    Error for_each_slave_transmit_start(std::size_t idx, ChannelIDT cid, NodeIDT nid, std::size_t len, const void* data, int timeout, std::tuple<SLAVE_IOS...>& tup)
    {
        return internal::slave_transmit<std::tuple_size<std::tuple<SLAVE_IOS...>>::value - 1, SLAVE_IOS...>{}(idx, cid, nid, len, data, timeout, tup);
    }
    
    template<typename MASTER_MSG_T, typename... SLAVE_IOS>
    Error for_each_slave_receive_start(std::size_t idx, ChannelIDT cid, MASTER_MSG_T& msg_out, int timeout, std::tuple<SLAVE_IOS...>& tup)
    {
        return internal::slave_receive<MASTER_MSG_T, std::tuple_size<std::tuple<SLAVE_IOS...>>::value - 1, SLAVE_IOS...>{}(idx, cid, msg_out, timeout, tup);
    }
    
    Error transmitToSlave(ChannelIDT cid, NodeIDT nid, std::size_t len, const void* data, int timeout)
    {
        int idx = -1;
        // find index
        for(std::size_t cid_idx = 0 ; cid_idx < SlaveCount ; ++cid_idx)
        {
            if(slave_channel_ids[cid_idx] == cid)
            {
                idx = cid_idx;
                break;
            }
        }
        if(idx != -1)
        {
            return for_each_slave_transmit_start((std::size_t)idx, cid, nid, len, data, timeout, slave_ios);
        }
        else
        {
            return Error::Unsupported;
        }
    }
    
    template<typename MASTER_MSG_T>
    Error receiveFromSlave(std::size_t idx, ChannelIDT cid, MASTER_MSG_T& msg_out, int timeout)
    {
        return for_each_slave_receive_start<MASTER_MSG_T>(idx, cid, msg_out, timeout, slave_ios);
    }
    
    IO_MASTER& master_io;
    
    std::array<ChannelIDT, SlaveCount> slave_channel_ids;
    std::tuple<typename IO_SLAVE_PAIRS::SlaveT&...> slave_ios;
};

}

#endif // LOWLEVELCOM_MUX_HPP
