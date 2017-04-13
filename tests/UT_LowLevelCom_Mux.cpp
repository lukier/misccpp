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
 */

// system
#include <stdint.h>
#include <stddef.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <exception>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <valarray>
#include <functional>

// testing framework & libraries
#include <gtest/gtest.h>

#include <misccpp/lowlevelcom/lowlevelcom.hpp>

class Test_LowLevelCom_Mux : public ::testing::Test
{
public:   
    Test_LowLevelCom_Mux()
    {
        
    }
    
    virtual ~Test_LowLevelCom_Mux()
    {
        
    }
};

TEST_F(Test_LowLevelCom_Mux, Dummy)
{   
    // Master
    typedef llc::transport::NanoMSGIO<true> NanoMSGIO;
    typedef typename NanoMSGIO::message_t nano_message_t;
    
    // Slave 1
    typedef llc::transport::lowlevel::USB LLUSB;
    typedef llc::transport::RawIO<LLUSB> USBIO;
    typedef typename USBIO::message_t usb_message_t;
    
    // Slave 2
    typedef llc::transport::lowlevel::SerialPort LLSP;
    typedef llc::transport::RawIO<LLSP> SPIO;
    typedef typename SPIO::message_t sp_message_t;
    
    typedef llc::MultiplexerLinux<NanoMSGIO, llc::SlaveChannelPair<USBIO, 5>, llc::SlaveChannelPair<SPIO,10>> MuxT;
    
    nn::socket_t nns;
    NanoMSGIO nmio(nns);
    
    LLUSB usbdev(0x1234,0x5678,0x02,0x82);
    USBIO usbio(usbdev);
    
    LLSP spdev("/dev/ttyACM0");
    SPIO spio(spdev);
    
    MuxT mux(nmio, usbio, spio);
}
