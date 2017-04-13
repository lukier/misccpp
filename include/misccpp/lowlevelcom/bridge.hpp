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
 * Interface to bridge two transports together.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_BRIDGE_HPP
#define LOWLEVELCOM_BRIDGE_HPP

#include <cstring>

#include <misccpp/lowlevelcom/utils.hpp>

namespace llc
{
    
template<typename IO_MASTER, typename IO_SLAVE>
class Bridge
{
public:
    typedef typename IO_MASTER::message_t master_message_t;
    typedef typename IO_SLAVE::message_t slave_message_t;
    
    static_assert(IO_MASTER::NeedsKnownInputMessage == false, "Unknown receive message not supported yet");
    static_assert(IO_SLAVE::NeedsKnownInputMessage == false, "Unknown receive message not supported yet");
    
    Bridge() = delete;
    Bridge(IO_MASTER& iom, IO_SLAVE& ios) : master_io(iom), slave_io(ios) { }
    virtual ~Bridge() { }
    
    // non copyable
    Bridge(const Bridge&) = delete;
    Bridge& operator=(Bridge const&) = delete; 
    
    // move constructor
    inline Bridge(Bridge&& m) noexcept : master_io(std::move(m.master_io)), slave_io(std::move(m.slave_io)) {  }
    
    // move assignment
    inline Bridge& operator=(Bridge&& m) noexcept { master_io = std::move(m.master_io); slave_io = std::move(m.slave_io); return *this; }
    
protected:
    template<typename _Rep1 = int64_t, typename _Period1 = std::ratio<1>, typename _Rep2 = int64_t, typename _Period2 = std::ratio<1>>
    Error forwardMasterToSlave(const std::chrono::duration<_Rep1, _Period1>& timeout_master = std::chrono::seconds(0), const std::chrono::duration<_Rep2, _Period2>& timeout_slave = std::chrono::seconds(0))
    {
        Error rc = Error::OK;
        
        master_message_t msg_in;
        rc = master_io.receive(msg_in, timeout_master);
        if(rc != Error::OK) { return rc; }
        
        slave_message_t msg_out(msg_in.size(), msg_in.getChannelID(), msg_in.getNodeID());
        if(msg_out.data() == 0) { return Error::OutOfMemory; }
        
        memcpy(msg_out.data(), msg_in.data(), msg_in.size()); // @note memcpy
        rc = slave_io.transmit(msg_out, timeout_slave);
        
        return rc;
    }
    
    template<typename _Rep1 = int64_t, typename _Period1 = std::ratio<1>, typename _Rep2 = int64_t, typename _Period2 = std::ratio<1>>
    Error forwardSlaveToMaster(const std::chrono::duration<_Rep1, _Period1>& timeout_master = std::chrono::seconds(0), const std::chrono::duration<_Rep2, _Period2>& timeout_slave = std::chrono::seconds(0))
    {
        Error rc = Error::OK;
        
        slave_message_t msg_in;
        rc = slave_io.receive(msg_in, timeout_slave);
        if(rc != Error::OK) { return rc; }
        
        master_message_t msg_out(msg_in.size(), msg_in.getChannelID(), msg_in.getNodeID());
        if(msg_out.data() == 0) { return Error::OutOfMemory; }
        
        memcpy(msg_out.data(), msg_in.data(), msg_in.size()); // @note memcpy
        rc = master_io.transmit(msg_out, timeout_master);
        return rc;
    }
private:
    IO_MASTER& master_io;
    IO_SLAVE& slave_io;
};

}

#endif // LOWLEVELCOM_BRIDGE_HPP
