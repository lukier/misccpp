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

#include <misccpp/drivers/SerialPort.hpp>

#include <string>
#include <chrono>
#include <stdexcept>
#include <unistd.h>
#include <termios.h>
#if defined(__linux__)
# include <linux/serial.h>
#endif
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <ctime>
#include <thread>
#include <mutex>
#include <condition_variable>

// Adapted from:
// https://github.com/bakercp/ofxSerial/

#ifndef TIOCINQ
#ifdef FIONREAD
#define TIOCINQ FIONREAD
#else
#define TIOCINQ 0x541B
#endif
#endif

static inline timespec timespec_from_ms(const uint32_t timeout_ms)
{
    timespec time;
    time.tv_sec = timeout_ms / 1e3;
    time.tv_nsec = (timeout_ms - (time.tv_sec * 1e3)) * 1e6;
    return time;
}

static inline void setXONXOFF(struct termios& options, bool v)
{
#ifdef IXANY
    if(v)
    {
        options.c_iflag |= (IXON | IXOFF);
    }
    else
    {
        options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
    }
#else // IXANY
    if (v)
    {
        options.c_iflag |= (IXON | IXOFF);
    }
    else
        options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
}
#endif // IXANY
}

static inline void setRTSCTS(struct termios& options, bool v)
{
#ifdef CRTSCTS
    if (v)
    {
        options.c_cflag |= (CRTSCTS);
    }
    else
    {
        options.c_cflag &= (unsigned long) ~(CRTSCTS);
    }
#elif defined CNEW_RTSCTS
    if(v)
    {
        options.c_cflag |=  (CNEW_RTSCTS);
    }
    else
    {
        options.c_cflag &= (unsigned long) ~(CNEW_RTSCTS);
    }
#else // CRTSCTS 
#error "OS Support seems wrong."
#endif
}


drivers::SerialPort::SerialPort() : port_fd(-1)
{

}

drivers::SerialPort::~SerialPort()
{

}

bool drivers::SerialPort::open(const char* ttydev, int baudrate, drivers::SerialPort::ByteSize bs, drivers::SerialPort::Parity par, drivers::SerialPort::StopBits sb, drivers::SerialPort::FlowControl fc)
{
    if(isOpen()) 
    {
        close();
    }
    
    port_fd = ::open(ttydev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    struct termios options;
    if(tcgetattr(port_fd, &options) == -1) 
    {
        ::close(port_fd);
        port_fd = -1;
        return isOpen();
    }
    
    options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
    options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN); //|ECHOPRT
    options.c_oflag &= (tcflag_t) ~(OPOST);
    options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
    options.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
    options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

    bool custom_baud = false;
    speed_t baud;
    switch (baudrate) 
    {
#ifdef B0
        case 0: baud = B0; break;
#endif
#ifdef B50
        case 50: baud = B50; break;
#endif
#ifdef B75
        case 75: baud = B75; break;
#endif
#ifdef B110
        case 110: baud = B110; break;
#endif
#ifdef B134
        case 134: baud = B134; break;
#endif
#ifdef B150
        case 150: baud = B150; break;
#endif
#ifdef B200
        case 200: baud = B200; break;
#endif
#ifdef B300
        case 300: baud = B300; break;
#endif
#ifdef B600
        case 600: baud = B600; break;
#endif
#ifdef B1200
        case 1200: baud = B1200; break;
#endif
#ifdef B1800
        case 1800: baud = B1800; break;
#endif
#ifdef B2400
        case 2400: baud = B2400; break;
#endif
#ifdef B4800
        case 4800: baud = B4800; break;
#endif
#ifdef B7200
        case 7200: baud = B7200; break;
#endif
#ifdef B9600
        case 9600: baud = B9600; break;
#endif
#ifdef B14400
        case 14400: baud = B14400; break;
#endif
#ifdef B19200
        case 19200: baud = B19200; break;
#endif
#ifdef B28800
        case 28800: baud = B28800; break;
#endif
#ifdef B57600
        case 57600: baud = B57600; break;
#endif
#ifdef B76800
        case 76800: baud = B76800; break;
#endif
#ifdef B38400
        case 38400: baud = B38400; break;
#endif
#ifdef B115200
        case 115200: baud = B115200; break;
#endif
#ifdef B128000
        case 128000: baud = B128000; break;
#endif
#ifdef B153600
        case 153600: baud = B153600; break;
#endif
#ifdef B230400
        case 230400: baud = B230400; break;
#endif
#ifdef B256000
        case 256000: baud = B256000; break;
#endif
#ifdef B460800
        case 460800: baud = B460800; break;
#endif
#ifdef B576000
        case 576000: baud = B576000; break;
#endif
#ifdef B921600
        case 921600: baud = B921600; break;
#endif
#ifdef B1000000
        case 1000000: baud = B1000000; break;
#endif
#ifdef B1152000
        case 1152000: baud = B1152000; break;
#endif
#ifdef B1500000
        case 1500000: baud = B1500000; break;
#endif
#ifdef B2000000
        case 2000000: baud = B2000000; break;
#endif
#ifdef B2500000
        case 2500000: baud = B2500000; break;
#endif
#ifdef B3000000
        case 3000000: baud = B3000000; break;
#endif
#ifdef B3500000
        case 3500000: baud = B3500000; break;
#endif
#ifdef B4000000
        case 4000000: baud = B4000000; break;
#endif
        default:
        {
            custom_baud = true;
#if defined(__linux__) && defined (TIOCSSERIAL)
            struct serial_struct ser;
            if(::ioctl(port_fd, TIOCGSERIAL, &ser) == -1) 
            {
                ::close(port_fd);
                port_fd = -1;
                return isOpen();
            }

            // set custom divisor
            ser.custom_divisor = ser.baud_base / static_cast<int> (baudrate);
            // update flags
            ser.flags &= ~ASYNC_SPD_MASK;
            ser.flags |= ASYNC_SPD_CUST;

            if(::ioctl (port_fd, TIOCSSERIAL, &ser) == -1) 
            {
                ::close(port_fd);
                port_fd = -1;
                return isOpen();
            }
#else
            ::close(port_fd);
            port_fd = -1;
            return isOpen();
#endif
        }
    }
    
    if(custom_baud == false) 
    {
#ifdef _BSD_SOURCE
        ::cfsetspeed(&options, baud);
#else
        ::cfsetispeed(&options, baud);
        ::cfsetospeed(&options, baud);
#endif
    }
    
    options.c_cflag &= (tcflag_t) ~CSIZE;
    switch(bs)
    {
        case drivers::SerialPort::ByteSize::B5: options.c_cflag |= CS5; break;
        case drivers::SerialPort::ByteSize::B6: options.c_cflag |= CS6; break;
        case drivers::SerialPort::ByteSize::B7: options.c_cflag |= CS7; break;
        case drivers::SerialPort::ByteSize::B8: options.c_cflag |= CS8; break;
        default:
        {
            ::close(port_fd);
            port_fd = -1;
            return isOpen();
        }
    }
    
    switch(sb)
    {
        case drivers::SerialPort::StopBits::One: options.c_cflag &= (tcflag_t) ~(CSTOPB); break;
        case drivers::SerialPort::StopBits::Two: options.c_cflag |=  (CSTOPB); break;
        default:
        {
            ::close(port_fd);
            port_fd = -1;
            return isOpen();
        }
    }
    
    options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
    switch(par)
    {
        case drivers::SerialPort::Parity::None: options.c_cflag &= (tcflag_t) ~(PARENB | PARODD); break;
        case drivers::SerialPort::Parity::Odd: options.c_cflag |= (PARENB | PARODD); break;
        case drivers::SerialPort::Parity::Even: options.c_cflag &= (tcflag_t) ~(PARODD); options.c_cflag |=  (PARENB); break;
#ifdef CMSPAR
        case drivers::SerialPort::Parity::Mark: options.c_cflag |= (PARENB | CMSPAR | PARODD); break;
        case drivers::SerialPort::Parity::Space: options.c_cflag |= (PARENB | CMSPAR); options.c_cflag &= (tcflag_t) ~(PARODD); break;
#endif // CMSPAR
        default:
        {
            ::close(port_fd);
            port_fd = -1;
            return isOpen();
        }
    }
    
    switch(fc)
    {
        case drivers::SerialPort::FlowControl::None: setXONXOFF(options, false); setRTSCTS(options, false); break;
        case drivers::SerialPort::FlowControl::Software: setXONXOFF(options, true); setRTSCTS(options, false); break;
        case drivers::SerialPort::FlowControl::Hardware: setXONXOFF(options, false); setRTSCTS(options, true); break;
        default:
        {
            ::close(port_fd);
            port_fd = -1;
            return isOpen();
        }
    }
    
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if(::tcsetattr(port_fd, TCSANOW, &options) == -1)
    {
        ::close(port_fd);
        port_fd = -1;
        return isOpen();
    }
    
    return isOpen();
}

bool drivers::SerialPort::isOpen()
{
    return port_fd != -1;
}

void drivers::SerialPort::close()
{
    if(isOpen())
    {
        ::close(port_fd);
        port_fd = -1;
    }
}

std::size_t drivers::SerialPort::haveMore()
{
    if(!isOpen()) { return 0; }
    
    int count = 0;
    if(::ioctl(port_fd, TIOCINQ, &count) == -1) 
    {
        return 0;
    } 
    else 
    {
        return static_cast<std::size_t>(count);
    }
}

bool drivers::SerialPort::waitForMoreImpl(const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(port_fd, &readfds);
    timespec timeout_ts (timespec_from_ms(timeout_ms));
    int r = pselect (port_fd + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

    if(r <= 0) // timeout or interrupt/error
    {
        return false;
    }
    
    if (!FD_ISSET(port_fd, &readfds)) 
    {
        return false;
    }
    
    return true;
}

bool drivers::SerialPort::readImpl(void* dst, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    uint8_t* dst_bytes = static_cast<uint8_t*>(dst);
    std::size_t bytes_read = 0;
    
    // read the immediately available ones
    {
        long int bytes_read_now = ::read(port_fd, dst_bytes, count);
        
        if(bytes_read_now >= 0) 
        {
            bytes_read = bytes_read_now;
        }
        else
        {
            return false; // error
        }
    }
    
    while(bytes_read < count) 
    {
        if(waitForMoreImpl(timeout_ms)) 
        {
            long int bytes_read_now = ::read(port_fd, dst_bytes + bytes_read, count - bytes_read);
            
            if(bytes_read_now < 0)
            {
                return false; // error
            }
            
            bytes_read += static_cast<std::size_t>(bytes_read_now);
            
            if(bytes_read == count) 
            {
                break;
            }
        
            if(bytes_read < count) 
            {
                continue;
            }
        }
    }
    
    return true;
}

bool drivers::SerialPort::writeImpl(const void* src, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    const uint8_t* src_bytes = static_cast<const uint8_t*>(src);
    fd_set writefds;
    std::size_t bytes_written = 0;
    
    while(bytes_written < count) 
    {
        timespec timeout(timespec_from_ms(timeout_ms));
        
        FD_ZERO(&writefds);
        FD_SET(port_fd, &writefds);

        int r = pselect(port_fd + 1, NULL, &writefds, NULL, &timeout, NULL);

        if(r <= 0) 
        {
            return false; // timeout or error/interrupt
        }
        
        if(FD_ISSET (port_fd, &writefds)) 
        {
            long int bytes_written_now = ::write(port_fd, src_bytes + bytes_written, count - bytes_written);
            
            if(bytes_written_now < 1)
            {
                return false;
            }
            
            bytes_written += static_cast<std::size_t> (bytes_written_now);
            
            if(bytes_written == count) 
            {
                break;
            }
            
            if(bytes_written < count) 
            {
                continue;
            }
        }
    }
    
    return true;
}

void drivers::SerialPort::flush()
{
    if(isOpen())
    {
        tcdrain(port_fd);
    }
}

void drivers::SerialPort::sendBreak(int duration)
{
    if(isOpen())
    {
        tcsendbreak(port_fd, static_cast<int> (duration / 4));
    }
}

void drivers::SerialPort::setRTS(bool v)
{
    if(isOpen())
    {
        setStatusBit(TIOCM_RTS, v);
    }
}

void drivers::SerialPort::setDTR(bool v)
{
    if(isOpen())
    {
        setStatusBit(TIOCM_DTR, v);
    }
}

bool drivers::SerialPort::getCTS()
{
    return getStatusBit(TIOCM_CTS);
}

bool drivers::SerialPort::getDSR()
{
    return getStatusBit(TIOCM_DSR);
}

bool drivers::SerialPort::getRI()
{
    return getStatusBit(TIOCM_RI);
}

bool drivers::SerialPort::getCD()
{
    return getStatusBit(TIOCM_CD);
}

void drivers::SerialPort::setStatusBit(int command, bool v)
{
    if(v) 
    {
        ::ioctl(port_fd, TIOCMBIS, &command);
    } 
    else 
    {
        ::ioctl(port_fd, TIOCMBIC, &command);
    }
}

bool drivers::SerialPort::getStatusBit(int bit)
{
    if(isOpen())
    {
        int status;

        if(::ioctl (port_fd, TIOCMGET, &status) != -1)
        {
            return (status & bit) != 0;
        }
    }
    
    return false;
}
