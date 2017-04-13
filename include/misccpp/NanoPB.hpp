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
 * C++ wrapper NanoPB.
 * ****************************************************************************
 */
#ifndef MISCCPP_NANOPB_HPP
#define MISCCPP_NANOPB_HPP

#include <cstdint>
#include <cstddef>
#include <cctype>

#include <pb_decode.h>
#include <pb_encode.h>

namespace npb
{
    
class InputStream : public pb_istream_t
{
public:
    InputStream() = delete;
    
    InputStream(const pb_byte_t *buf, std::size_t bufsize) : pb_istream_t(pb_istream_from_buffer(buf, bufsize))
    {
        
    }
    
    virtual ~InputStream()
    {
        
    }
    
    InputStream(const InputStream&) = default;
    InputStream(InputStream&&) = default;
    InputStream& operator=(const InputStream&) = default;
    InputStream& operator=(InputStream&&) = default;
    
    std::size_t bytesLeft() const { return pb_istream_t::bytes_left; }
    
    bool read(pb_byte_t *buf, std::size_t count)
    {
        return pb_read(this, buf, count);
    }
};

class OutputStream : public pb_ostream_t
{
public:
    OutputStream() = delete;
    
    OutputStream(pb_byte_t *buf, std::size_t bufsize) : pb_ostream_t(pb_ostream_from_buffer(buf, bufsize))
    {
        
    }
    
    virtual ~OutputStream()
    {
        
    }
    
    OutputStream(const OutputStream&) = default;
    OutputStream(OutputStream&&) = default;
    OutputStream& operator=(const OutputStream&) = default;
    OutputStream& operator=(OutputStream&&) = default;
    
    std::size_t bytesWritten() const { return pb_ostream_t::bytes_written; }
    std::size_t totalSize() const { return pb_ostream_t::max_size; }
    
    bool write(const pb_byte_t *buf, std::size_t count)
    {
        return pb_write(this, buf, count);
    }
};    
    
}

#define NANOPB_CXX_WRAPPER(CLASS_NAME)\
class CLASS_NAME : public _##CLASS_NAME\
{\
public:\
    CLASS_NAME() : _##CLASS_NAME(CLASS_NAME##_init_default) { }\
    CLASS_NAME(const _##CLASS_NAME& other) : _##CLASS_NAME(other) { }\
    static CLASS_NAME zero() { return CLASS_NAME(CLASS_NAME##_init_zero); }\
    virtual ~CLASS_NAME()  { }\
    CLASS_NAME(const CLASS_NAME&) = default;\
    CLASS_NAME(CLASS_NAME&&) = default;\
    CLASS_NAME& operator=(const CLASS_NAME&) = default;\
    CLASS_NAME& operator=(CLASS_NAME&&) = default;\
    \
    bool decode(npb::InputStream& is) { return pb_decode(&is, CLASS_NAME##_fields, this); }\
    bool encode(npb::OutputStream& os) const { return pb_encode(&os, CLASS_NAME##_fields, this); }\
    std::size_t getEncodedSize() const { std::size_t ret = 0; pb_get_encoded_size(&ret, CLASS_NAME##_fields, this); return ret; }\
}

#endif // MISCCPP_NANOPB_HPP
