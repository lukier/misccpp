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
 * Generic library for packet based interfacing between various nodes.
 * ****************************************************************************
 */

#include <misccpp/lowlevelcom/mux.hpp>
#include <misccpp/lowlevelcom/bridge.hpp>
#include <misccpp/lowlevelcom/cobs.hpp>
#include <misccpp/lowlevelcom/transport/transport_raw.hpp>

#ifdef __linux__
#include <misccpp/lowlevelcom/boost_crc.hpp>
#include <misccpp/lowlevelcom/bridge_linux.hpp>
#include <misccpp/lowlevelcom/mux_linux.hpp>
#include <misccpp/lowlevelcom/transport/transport_can.hpp>
#include <misccpp/lowlevelcom/transport/transport_i2c.hpp>
#include <misccpp/lowlevelcom/transport/transport_nanomsg.hpp>
#include <misccpp/lowlevelcom/transport/transport_netlink.hpp>
#include <misccpp/lowlevelcom/transport/transport_serial_port.hpp>
#include <misccpp/lowlevelcom/transport/transport_spi.hpp>
#include <misccpp/lowlevelcom/transport/transport_usb.hpp>
#include <misccpp/lowlevelcom/transport/transport_zeromq.hpp>
#endif // __linux__
