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
 * Simple C++ interface to spi-dev.
 * ****************************************************************************
 */

#include <misccpp/drivers/DevSPI.hpp>

#include <string>
#include <chrono>
#include <stdexcept>
#include <cstring>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

static inline timespec timespec_from_ms(const uint32_t timeout_ms)
{
    timespec time;
    time.tv_sec = timeout_ms / 1e3;
    time.tv_nsec = (timeout_ms - (time.tv_sec * 1e3)) * 1e6;
    return time;
}

drivers::DevSPI::DevSPI() : port_fd(-1)
{

}

drivers::DevSPI::~DevSPI()
{

}

bool drivers::DevSPI::open(const char* spidev, uint32_t freq, uint8_t bpw, bool cpha, bool cpol, bool lsb_first)
{
    port_fd = ::open(spidev, O_RDWR);
    if(port_fd < 0) return false;
    
    if(::ioctl(port_fd, SPI_IOC_RD_BITS_PER_WORD, &bpw) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    if(::ioctl(port_fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    if(::ioctl(port_fd, SPI_IOC_RD_MAX_SPEED_HZ, &freq) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    if(::ioctl(port_fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    const uint8_t endian_val = (lsb_first == true ? 1 : 0);
    if(::ioctl(port_fd, SPI_IOC_RD_LSB_FIRST, &endian_val) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    if(::ioctl(port_fd, SPI_IOC_WR_LSB_FIRST, &endian_val) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    uint8_t spi_mode = 0;
    
    if(cpha == false && cpol == false)
    {
        spi_mode = SPI_MODE_0;
    }
    else if(cpha == true && cpol == false)
    {
        spi_mode = SPI_MODE_1;
    }
    else if(cpha == false && cpol == true)
    {
        spi_mode = SPI_MODE_2;
    }
    else if(cpha == true && cpol == true)
    {
        spi_mode = SPI_MODE_3;
    }
    
    if(::ioctl(port_fd, SPI_IOC_RD_MODE, &spi_mode) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    if(::ioctl(port_fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
    {
        ::close(port_fd);
        port_fd = -1;
        return false;
    }
    
    return true;
}

bool drivers::DevSPI::isOpen()
{
    return port_fd != -1;
}

void drivers::DevSPI::close()
{
    if(isOpen())
    {
        ::close(port_fd);
        port_fd = -1;
    }
}

bool drivers::DevSPI::readImpl(void* dst, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    struct spi_ioc_transfer spi_message[1];
    ::memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].rx_buf = (uintptr_t)dst;
    spi_message[0].len = count;
    
    if(::ioctl(port_fd, SPI_IOC_MESSAGE(1), spi_message) == -1)
    {
        return false;
    }
     
    return true;
}

bool drivers::DevSPI::writeImpl(const void* src, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    struct spi_ioc_transfer spi_message[1];
    ::memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].tx_buf = (uintptr_t)src;
    spi_message[0].len = count;

    if(::ioctl(port_fd, SPI_IOC_MESSAGE(1), spi_message) == -1)
    {
        return false;
    }
    
    return true;
}

bool drivers::DevSPI::readWriteImpl(const void* data_out, void* data_in, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return false; }
    
    struct spi_ioc_transfer spi_message[1];
    ::memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].tx_buf = (uintptr_t)data_out;
    spi_message[0].rx_buf = (uintptr_t)data_in;
    spi_message[0].len = count;
    
    if(::ioctl(port_fd, SPI_IOC_MESSAGE(1), spi_message) == -1)
    {
        return false;
    }
    
    return true;
}
