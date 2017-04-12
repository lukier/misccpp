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
 * Linux threads based two transport bridge.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_BRIDGE_LINUX_HPP
#define LOWLEVELCOM_BRIDGE_LINUX_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdexcept>
#include <atomic>
#include <thread>

#include <misccpp/lowlevelcom/bridge.hpp>

/// @todo This should employ poll when using ZeroMQ/NanoMSG

namespace llc
{

template<typename IO_MASTER, typename IO_SLAVE>
class BridgeLinux : public Bridge<IO_MASTER,IO_SLAVE>
{
    typedef Bridge<IO_MASTER,IO_SLAVE> BaseBridgeT;
public:
    BridgeLinux() = delete;
    BridgeLinux(IO_MASTER& iom, IO_SLAVE& ios) : BaseBridgeT(iom, ios), time_to_end(true)
    {
        
    }
    
    virtual ~BridgeLinux()
    {
        stop();
    }
    
    bool isRunning() const { return time_to_end == false; }
    
    void start()
    {
        if(time_to_end == false) { return; }  // already running
        
        time_to_end = false;
        thr = std::thread(&BridgeLinux::broker, this);
    }
    
    void stop()
    {
        if(time_to_end == false)
        {
            time_to_end = true;
            thr.join();
        }
    }
    
    int getTimeoutMaster() const { return timeout_ms; }
    int getTimeoutSlaves() const { return timeout_sl; }
    
    void setTimeouts(int tms, int tsl)
    {
        timeout_ms = tms;
        timeout_sl = tsl;
    }
private:
    void broker()
    {
        while(!time_to_end)
        {
            try
            {
                BaseBridgeT::forwardSlaveToMaster(timeout_ms,timeout_sl);
                BaseBridgeT::forwardMasterToSlave(timeout_ms,timeout_sl);
            }
            catch(const std::exception& ex)
            {
                // TODO FIXME and return codes as well
            }
        }
    }
    
    std::atomic<bool> time_to_end;
    std::thread thr;
    int timeout_ms, timeout_sl;
};

}

#endif // LOWLEVELCOM_BRIDGE_LINUX_HPP
