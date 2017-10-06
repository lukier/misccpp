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
 * Simple C++ interface to libusb.
 * ****************************************************************************
 */

#ifndef DRIVERS_DEV_USB_HPP
#define DRIVERS_DEV_USB_HPP

#include <cstdint>
#include <chrono>

struct libusb_context;
struct libusb_device_handle;
struct libusb_device;

namespace drivers
{

/**
 * libusb Interface.
 */    
class DevUSB
{
public:
    DevUSB();
    virtual ~DevUSB();
    
    bool open(uint16_t a_vid, uint16_t a_pid, uint8_t a_ep_in = 0xFF, uint8_t a_ep_out = 0xFF);
    bool isOpen();
    void close();
    
    inline uint8_t getEndpointIN() const { return ep_in; }
    inline uint8_t getEndpointOUT() const { return ep_out; }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    int readControl(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, void* dst, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return readControlImpl(bmRequestType, bRequest, wValue, wIndex, dst, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    int writeControl(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, const void* src, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return writeControlImpl(bmRequestType, bRequest, wValue, wIndex, src, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }

    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    int read(void* dst, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return readBulkImpl(dst, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    int write(const void* src, std::size_t count, const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return writeBulkImpl(src, count, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }

private:
    int readBulkImpl(void* dst, std::size_t count, const uint32_t timeout_ms);
    int writeBulkImpl(const void* src, std::size_t count, const uint32_t timeout_ms);
    int readControlImpl(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, void* dst, std::size_t count, const uint32_t timeout_ms);
    int writeControlImpl(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, const void* src, std::size_t count, const uint32_t timeout_ms);
    
    uint8_t ep_in, ep_out;
    int interface_number;
    libusb_context *usbctx;
    libusb_device_handle* devh;
    libusb_device* dev;
};
    
}

#endif // DRIVERS_DEV_USB_HPP
