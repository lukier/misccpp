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
 * Trivial binary buffer decoder. Assumes packed.
 * ****************************************************************************
 */

#ifndef BUFFER_DECODE_HPP
#define BUFFER_DECODE_HPP

#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <stdexcept>

class BufferDecode
{
public:
    BufferDecode(const void* buffer, std::size_t size, std::size_t init_offset = 0)
    :   m_data(static_cast<const uint8_t*>(buffer)), 
        m_max_size(size), 
        m_ptr(init_offset), 
        m_init_offset(init_offset) { }
    
    void reset() { m_ptr = m_init_offset; }
    void skip(std::size_t bytes) { m_ptr += bytes; } 
    
    template<typename T>
    T read()
    {
        T ret;
        if((m_ptr - m_init_offset) + sizeof(T) > m_max_size)
        {
            eof();
        }
        memcpy(&ret, &m_data[m_ptr], sizeof(T));
        m_ptr += sizeof(T);
        return ret;
    }
    
    template<typename T>
    void read(T& ret)
    {
        if((m_ptr - m_init_offset) + sizeof(T) > m_max_size)
        {
            eof();
        }
        memcpy(&ret, &m_data[m_ptr], sizeof(T));
        m_ptr += sizeof(T);
    }
    
    void read(void* buffer, std::size_t size)
    {
        if((m_ptr - m_init_offset) + size > m_max_size)
        {
            eof();
        }
        memcpy(buffer, &m_data[m_ptr], size);
        m_ptr += size;
    }
    
    void read(std::string& s)
    {
        char c;
        while(1)
        {
            // check eof
            if((m_ptr - m_init_offset) > m_max_size)
            {
                eof();
            }
            
            // read char
            c = m_data[m_ptr];
            ++m_ptr;
            
            // check null terminator
            if(c == 0)
            {
                break;
            }
            else // append to the string
            {
                s.append(&c, 1);
            }
        }
    }
private:
    void eof()
    {
        throw std::runtime_error("Not enough data");
    }
    
    const uint8_t* m_data;
    std::size_t m_max_size;
    std::size_t m_ptr;
    std::size_t m_init_offset;
};

#endif // BUFFER_DECODE_HPP
