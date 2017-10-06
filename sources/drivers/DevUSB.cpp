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

#include <misccpp/drivers/DevUSB.hpp>

#include <string>
#include <chrono>
#include <stdexcept>
#include <cstring>

#pragma GCC diagnostic push 
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-pedantic"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#include <libusb.h>
#pragma GCC diagnostic pop

drivers::DevUSB::DevUSB() : ep_in(0xFF), ep_out(0xFF), interface_number(-1), usbctx(0), devh(0), dev(0)
{

}

drivers::DevUSB::~DevUSB()
{

}

bool drivers::DevUSB::open(uint16_t a_vid, uint16_t a_pid, uint8_t a_ep_in, uint8_t a_ep_out)
{
    int r = libusb_init(&usbctx); //initialize a library session
    if(r < 0) 
    {
        return false;
    }
    
    libusb_set_debug(usbctx, 3); //set verbosity level to 3, as suggested in the documentation
    
    devh = libusb_open_device_with_vid_pid(usbctx, a_vid, a_pid);
    
    if(devh == 0)
    {
        libusb_exit(usbctx); //close the session
        usbctx = 0;
        return false;
    }
    
    dev = libusb_get_device(devh);
    
    if(dev == 0)
    {
        libusb_exit(usbctx); //close the session
        usbctx = 0;
        return false;
    }
    
    // get endpoints for our interface
    libusb_config_descriptor *config;
    
    libusb_get_config_descriptor(dev, 0, &config);
    
    const libusb_interface *inter;
    const libusb_interface_descriptor *interdesc;
    const libusb_endpoint_descriptor *epdesc;
    
    for(int i = 0; i < (int)config->bNumInterfaces; i++) 
    {
        inter = &config->interface[i];
        for(int j = 0; j < inter->num_altsetting; j++) 
        {
            interdesc = &inter->altsetting[j];
            if(interdesc->bInterfaceClass == 0xFF) // vendor
            {
                interface_number = interdesc->bInterfaceNumber;
                
                for(int k = 0; k < (int)interdesc->bNumEndpoints; k++)
                {
                    epdesc = &interdesc->endpoint[k];
                    
                    // OUT
                    if(a_ep_out != 0xFF)
                    {
                        if(((epdesc->bmAttributes & 0x03) == 2) && ((epdesc->bEndpointAddress & 0x80) == 0) && (epdesc->bEndpointAddress == a_ep_out)) // BULK and OUT
                        {
                            ep_out = a_ep_out;
                        }
                    }
                    
                    // IN
                    if(a_ep_in != 0xFF)
                    {
                        if(((epdesc->bmAttributes & 0x03) == 2) && ((epdesc->bEndpointAddress & 0x80) != 0) && (epdesc->bEndpointAddress == a_ep_in)) // BULK and IN
                        {
                            ep_in = a_ep_in;
                        }
                    }
                }
            }
        }
    }
    
    libusb_free_config_descriptor(config);
    
    if(interface_number >= 0)
    {
        r = libusb_claim_interface(devh, interface_number); //claim interface
        if(r < 0) 
        {
            libusb_close(devh);
            libusb_exit(usbctx); //close the session
            usbctx = 0;
            return false;
        }
    }
    else
    {
        libusb_close(devh);
        libusb_exit(usbctx); //close the session
        usbctx = 0;
        return false;
    }
    
    if((ep_in == 0xFF) && (ep_out == 0xFF))
    {
        libusb_release_interface(devh, interface_number);
        libusb_close(devh);
        libusb_exit(usbctx); //close the session
        usbctx = 0;
        return false;
    }
    
    return true;
}

bool drivers::DevUSB::isOpen()
{
    return usbctx != 0;
}

void drivers::DevUSB::close()
{
    if(isOpen())
    {
        libusb_release_interface(devh, interface_number);
        libusb_close(devh);
        libusb_exit(usbctx); //close the session
        usbctx = 0;
    }
}

int drivers::DevUSB::readBulkImpl(void* dst, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return -1; }
    
    if(ep_in != 0xFF)
    {
        int rx_actual_transfer = 0;
        int r = libusb_bulk_transfer(devh, ep_in, (unsigned char*)dst, count, &rx_actual_transfer, timeout_ms);
        if(r == 0)
        {
            return rx_actual_transfer;
        }
        else
        {
            return r;
        }
    }
    else
    {
        return -1;
    }
     
    return -1;
}

int drivers::DevUSB::writeBulkImpl(const void* src, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return -1; }
    
    if(ep_out != 0xFF)
    {
        int tx_actual_transfer = 0;
        int r = libusb_bulk_transfer(devh, ep_out, (unsigned char*)src, count, &tx_actual_transfer, timeout_ms);
        if(r == 0)
        {
            return tx_actual_transfer;
        }
        else
        {
            return r;
        }
    }
    else
    {
        return -1;
    }
    
    return -1;
}

int drivers::DevUSB::readControlImpl(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, void* dst, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return -1; }
    
    return libusb_control_transfer(devh, LIBUSB_ENDPOINT_IN | bmRequestType, bRequest, wValue, wIndex, (unsigned char*)dst, count, timeout_ms);
}

int drivers::DevUSB::writeControlImpl(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, const void* src, std::size_t count, const uint32_t timeout_ms)
{
    if(!isOpen()) { return -1; }
    
    return libusb_control_transfer(devh, LIBUSB_ENDPOINT_OUT | bmRequestType, bRequest, wValue, wIndex, (unsigned char*)src, count, timeout_ms);
}

