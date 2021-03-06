/**
 * ****************************************************************************
 * Copyright (c) 2016, Robert Lukierski.
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
 * Simple C++ serial port driver
 * ****************************************************************************
 */

#ifndef DRIVERS_SERIAL_PORT_HPP
#define DRIVERS_SERIAL_PORT_HPP

#include <cstdint>
#include <chrono>

namespace drivers
{
    
class SerialPort
{
public:
    enum class ByteSize
    {
        B5,
        B6,
        B7,
        B8,
    };
    
    enum class Parity
    {
        None,
        Odd,
        Even,
        Mark,
        Space
    };
    
    enum class StopBits
    {
        One,
        Two,
    };
    
    enum class FlowControl
    {
        None,
        Software,
        Hardware
    };
    
    SerialPort();
    virtual ~SerialPort();
    
    bool open(const char* ttydev, int baudrate = 115200, ByteSize bs = ByteSize::B8, Parity par = Parity::None, StopBits sb = StopBits::One, FlowControl fc = FlowControl::None);
    bool isOpen();
    void close();
    
    std::size_t haveMore();
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool waitForMore(const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return waitForMoreImpl(std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }

    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool read(void* dst, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return readImpl(dst, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool write(const void* src, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return writeImpl(src, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
    void flush();
    void sendBreak(int duration);
    void setRTS(bool v);
    void setDTR(bool v);
    bool getCTS();
    bool getDSR();
    bool getRI();
    bool getCD();
private:
    bool waitForMoreImpl(const uint32_t timeout_ms);
    bool readImpl(void* dst, std::size_t count, const uint32_t timeout_ms);
    bool writeImpl(const void* src, std::size_t count, const uint32_t timeout_ms);
    bool getStatusBit(int bit);
    void setStatusBit(int command, bool v);
    int port_fd;
};
    
}

#endif // DRIVERS_SERIAL_PORT_HPP
