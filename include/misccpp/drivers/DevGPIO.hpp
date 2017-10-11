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

#ifndef DRIVERS_DEV_GPIO_HPP
#define DRIVERS_DEV_GPIO_HPP

#include <cstdint>
#include <chrono>

namespace drivers
{
    
enum GPIOPort
{
    GPIOPortA = 0,
    GPIOPortB,
    GPIOPortC,
    GPIOPortD,
    GPIOPortE,
    GPIOPortF,
    GPIOPortG,
    GPIOPortH,
    GPIOPortI,
    GPIOPortJ
};

enum GPIODirection
{
    GPIODirectionIn = 0,
    GPIODirectionOut
};

enum GPIOTrigger
{
    GPIOTriggerNone = 0,
    GPIOTriggerRising,
    GPIOTriggerFalling,
    GPIOTriggerBoth
};

enum GPIOLogicActive
{
    GPIOLogicActiveHigh = 0,
    GPIOLogicActiveLow
};

/**
 * GPIO Dev Interface.
 */    
class DevGPIO
{
public:
    DevGPIO();
    virtual ~DevGPIO();
    
    bool open(std::size_t pn);
    bool open(GPIOPort port, std::size_t pn);
    inline bool isOpen() const { return is_exported; }
    bool close();
    
    bool getValue() const;
    void setValue(bool b);
    
    GPIODirection getDirection() const;
    void setDirection(GPIODirection dir);
    
    GPIOTrigger getTrigger() const;
    void setTrigger(GPIOTrigger t);
    
    template<typename _Rep = int64_t, typename _Period = std::ratio<1>>
    bool waitForTrigger(const std::chrono::duration<_Rep, _Period>& timeout = std::chrono::seconds(0))
    {
        return waitForTriggerImpl(std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    }
    
    GPIOLogicActive getLogicActive() const;
    void setLogicActive(GPIOLogicActive la);
private:
    bool waitForTrigger(const uint32_t timeout_ms);
    bool exportPin();
    bool unexportPin();
    bool is_exported;
    std::size_t pin_num;
};
    
}

#endif // DRIVERS_DEV_GPIO_HPP
