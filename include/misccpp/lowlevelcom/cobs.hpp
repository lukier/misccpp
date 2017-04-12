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
 * Consistent Overhead Byte Stuffing implementation.
 * ****************************************************************************
 */
#ifndef LOWLEVELCOM_COBS_HPP
#define LOWLEVELCOM_COBS_HPP

#include <cstdint>
#include <cstddef>
#include <utility>

namespace llc
{

    struct COBS
    {
        static constexpr uint8_t Delimiter = 0;
        
        // add overhead
        static constexpr inline std::size_t calculateNewSize(std::size_t s) 
        { 
            return (s < 254) ? (s + 1) : (s + ((s / 254))); 
        }
        
        static std::size_t encode(const void* buf_in, std::size_t length, void* buf_out)
        {
            const uint8_t* input = static_cast<const uint8_t*>(buf_in);
            uint8_t* output = static_cast<uint8_t*>(buf_out);
            
            std::size_t read_index = 0;
            std::size_t write_index = 1;
            std::size_t code_index = 0;
            uint8_t code = 1;
            
            while(read_index < length)
            {
                if(input[read_index] == Delimiter)
                {
                    output[code_index] = code;
                    code = 1;
                    code_index = write_index++;
                    read_index++;
                }
                else
                {
                    output[write_index++] = input[read_index++];
                    code++;
                    if(code == 0xFF)
                    {
                        output[code_index] = code;
                        code = 1;
                        code_index = write_index++;
                    }
                }
            }
            
            output[code_index] = code;
            
            return write_index;
        }
        
        static std::size_t decode(const void* buf_in, std::size_t length, void* buf_out)
        {
            const uint8_t* input = static_cast<const uint8_t*>(buf_in);
            uint8_t* output = static_cast<uint8_t*>(buf_out);
            
            std::size_t read_index = 0;
            std::size_t write_index = 0;
            uint8_t code;
            uint8_t i;
            
            while(read_index < length)
            {
                code = input[read_index];
                
                if(read_index + code > length && code != 1)
                {
                    return 0;
                }
                
                read_index++;
                
                for(i = 1; i < code; i++)
                {
                    output[write_index++] = input[read_index++];
                }
                if(code != 0xFF && read_index != length)
                {
                    output[write_index++] = Delimiter;
                }
            }
            
            return write_index;
        }
    };
}
#endif // LOWLEVELCOM_COBS_HPP
