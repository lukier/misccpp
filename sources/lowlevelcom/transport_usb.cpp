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
 * Linux libUSB based low level transport.
 * ****************************************************************************
 */
#include <stdexcept>
#include <libusb.h>

#include <misccpp/lowlevelcom/transport/transport_usb.hpp>

llc::transport::lowlevel::USB::USB(uint16_t a_vid, uint16_t a_pid, uint8_t a_ep_in, uint8_t a_ep_out) : ep_in(0xFF), ep_out(0xFF), interface_number(-1)
{
    int r = libusb_init(&usbctx); //initialize a library session
    if(r < 0) 
    {
        throw std::runtime_error("libusb init error");
    }
    
    libusb_set_debug(usbctx, 3); //set verbosity level to 3, as suggested in the documentation
    
    devh = libusb_open_device_with_vid_pid(usbctx, a_vid, a_pid);
    
    if(devh == 0)
    {
        libusb_exit(usbctx); //close the session
        usbctx = 0;
        throw std::runtime_error("No libusb device");
    }
    
    dev = libusb_get_device(devh);
    
    if(dev == 0)
    {
        libusb_exit(usbctx); //close the session
        usbctx = 0;
        throw std::runtime_error("libusb get_device error");
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
            throw std::runtime_error("Cannot claim interface");
        }
    }
    else
    {
        libusb_close(devh);
        libusb_exit(usbctx); //close the session
        throw std::runtime_error("No vendor interface in the device");
    }
    
    if((ep_in == 0xFF) && (ep_out == 0xFF))
    {
        libusb_release_interface(devh, interface_number);
        libusb_close(devh);
        libusb_exit(usbctx); //close the session
        throw std::runtime_error("No bulk endpoints in the interface at all");
    }
}

llc::transport::lowlevel::USB::~USB()
{
    libusb_release_interface(devh, interface_number);
    libusb_close(devh);
    libusb_exit(usbctx); //close the session
}

llc::Error llc::transport::lowlevel::USB::transmit(const void* ptr, std::size_t len, int timeout)
{
    if(ep_out != 0xFF)
    {
        int tx_actual_transfer = 0;
        int r = libusb_bulk_transfer(devh, ep_out, (unsigned char*)ptr, len, &tx_actual_transfer, timeout);
        if((r == 0) && (tx_actual_transfer == (int)len))
        {
            return Error::OK;
        }
        else
        {
            return Error::Timeout;
        }
    }
    else
    {
        return Error::Unsupported;
    }
}

llc::Error llc::transport::lowlevel::USB::transmitStart(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    if(ep_out != 0xFF) { return Error::OK; } else { return Error::Unsupported; }
}

llc::Error llc::transport::lowlevel::USB::transmitReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::transmitComplete(llc::ChannelIDT chan_id, llc::NodeIDT node_id, int timeout)
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::receive(void* ptr, std::size_t len, int timeout)
{
    if(ep_in != 0xFF)
    {
        int rx_actual_transfer = 0;
        int r = libusb_bulk_transfer(devh, ep_in, (unsigned char*)ptr, len, &rx_actual_transfer, timeout);
        if((r == 0) && (rx_actual_transfer == (int)len))
        {
            return Error::OK;
        }
        else
        {
            return Error::Timeout;
        }
    }
    else
    {
        return Error::Unsupported;
    }
}

llc::Error llc::transport::lowlevel::USB::receiveStart(int timeout)
{
    if(ep_in != 0xFF) { return Error::OK; } else { return Error::Unsupported; }
}

llc::Error llc::transport::lowlevel::USB::receiveReset()
{
    return Error::OK;
}

llc::Error llc::transport::lowlevel::USB::receiveComplete(int timeout)
{
    return Error::OK;
}
