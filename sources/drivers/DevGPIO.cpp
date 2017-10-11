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
 * Simple C++ interface to gpiolib.
 * ****************************************************************************
 */

#include <misccpp/drivers/DevGPIO.hpp>

#include <string>
#include <sstream>
#include <chrono>
#include <stdexcept>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>

#include <misccpp/File.hpp>

static inline timespec timespec_from_ms(const uint32_t timeout_ms)
{
    timespec time;
    time.tv_sec = timeout_ms / 1e3;
    time.tv_nsec = (timeout_ms - (time.tv_sec * 1e3)) * 1e6;
    return time;
}

static constexpr const char* SYSFS_DIR = "/sys/class/gpio";

drivers::DevGPIO::DevGPIO() : is_exported(false), pin_num(0)
{
    
}

drivers::DevGPIO::~DevGPIO()
{
    if(is_exported)
    {
        unexportPin();
    }
}

bool drivers::DevGPIO::open(std::size_t pn)
{
    pin_num = pn;
    return exportPin();
}

bool drivers::DevGPIO::open(GPIOPort port, std::size_t pn)
{
    return open((std::size_t)port * 32 + pn);
}

bool drivers::DevGPIO::close()
{
    if(is_exported)
    {
        return unexportPin();
    }
    
    return true;
}

bool drivers::DevGPIO::getValue() const
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/value";
    
    File fd(ss.str().c_str(),"r");
    char c = fd.getc();
    return c == '1';
}

void drivers::DevGPIO::setValue(bool b)
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/value";
    
    File fd(ss.str().c_str(),"w");
    if(b)
    {
        fd.putc('1');
    }
    else
    {
        fd.putc('0');
    }
}

drivers::GPIODirection drivers::DevGPIO::getDirection() const
{
    char buf[4];
    
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/direction";
    
    File fd(ss.str().c_str(),"r");
    fd.scanf("%s", buf);
    
    // let's do tolower to be sure
    for (char* p = buf ; *p ; ++p) { *p = tolower(*p); }
    
    if(strcmp(buf,"out") == 0)
    {
        return drivers::GPIODirectionOut;
    }
    else
    {
        return drivers::GPIODirectionIn;
    }
}

void drivers::DevGPIO::setDirection(drivers::GPIODirection dir)
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/direction";
    
    File fd(ss.str().c_str(),"w");
    
    if(dir == GPIODirectionIn)
    {
        fd.puts("in");
    }
    else
    {
        fd.puts("out");
    }
}

drivers::GPIOTrigger drivers::DevGPIO::getTrigger() const
{
    char buf[8];
    
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/edge";
    
    File fd(ss.str().c_str(),"r");
    fd.scanf("%s", buf);
    
    // let's do tolower to be sure
    for (char* p = buf ; *p ; ++p) { *p = tolower(*p); }
    
    if(strcmp(buf,"rising") == 0)
    {
        return drivers::GPIOTriggerRising;
    }
    else if(strcmp(buf,"falling") == 0)
    {
        return drivers::GPIOTriggerFalling;
    }
    else if(strcmp(buf,"both") == 0)
    {
        return drivers::GPIOTriggerBoth;
    }
    else
    {
        return drivers::GPIOTriggerNone;
    }
}

void drivers::DevGPIO::setTrigger(drivers::GPIOTrigger t)
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/edge";
    
    File fd(ss.str().c_str(),"w");
    
    switch(t)
    {
        case drivers::GPIOTriggerNone: fd.puts("none"); break;
        case drivers::GPIOTriggerRising:  fd.puts("rising"); break;
        case drivers::GPIOTriggerFalling:  fd.puts("falling"); break;
        case drivers::GPIOTriggerBoth:  fd.puts("both"); break;
    }
}

drivers::GPIOLogicActive drivers::DevGPIO::getLogicActive() const
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/active_low";
    
    File fd(ss.str().c_str(),"r");
    char c = fd.getc();
    
    return (c == '1' ? drivers::GPIOLogicActiveLow : drivers::GPIOLogicActiveHigh);
}

void drivers::DevGPIO::setLogicActive(drivers::GPIOLogicActive la)
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/active_low";
    
    File fd(ss.str().c_str(),"w");
    if(la == GPIOLogicActiveLow)
    {
        fd.putc('1');
    }
    else
    {
        fd.putc('0');
    }
}

bool drivers::DevGPIO::waitForTrigger(const uint32_t timeout_ms)
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/gpio" << pin_num << "/value";
    
    int fd = ::open(ss.str().c_str(), O_RDONLY | O_NONBLOCK);
    
    if(fd < 0) { return false; }
    
    struct pollfd fdset[1];
    int nfds = 1;
    
    memset((void*)fdset, 0, sizeof(fdset));

    fdset[0].fd = fd;
    fdset[0].events = POLLPRI;
    
    int new_timeout_ms = timeout_ms;
    if(new_timeout_ms == 0) { new_timeout_ms = -1; } // negative = infinite

    int rc = poll(fdset, nfds, new_timeout_ms);
    if(rc < 0) { return false; }
    
    if (fdset[0].revents & POLLPRI) 
    {
        lseek(fdset[0].fd, 0, SEEK_SET);
        char c;
        int len = read(fdset[0].fd, &c, 1); // read new value, unused
    }
    
    ::close(fd);
    
    return true;
}

bool drivers::DevGPIO::exportPin()
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/export";
    
    File fd(ss.str().c_str(),"w");
    int rc = fd.printf("%d", pin_num);
    
    if(rc > 0)
    {
        is_exported = true;
        return true;
    }
    else
    {
        is_exported = false;
        return false;
    }
}

bool drivers::DevGPIO::unexportPin()
{
    std::stringstream ss;
    ss << SYSFS_DIR << "/unexport";
    
    File fd(ss.str().c_str(),"w");
    int rc = fd.printf("%d", pin_num);
    
    return rc > 0;
}
