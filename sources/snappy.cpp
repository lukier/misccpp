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
 * Snappy Compression C++ Wrappers.
 * ****************************************************************************
 */

#include <cstdint>
#include <cstddef>

#include <snappy.h>

#include <misccpp/snappy.hpp>

std::size_t snappy::maxCompressedSize(std::size_t uncomp_len)
{
    return snappy::MaxCompressedLength(uncomp_len);
}

std::size_t snappy::compress(const void* uncomp_ptr, std::size_t uncomp_len, void* comp_ptr)
{
    std::size_t comp_size = 0;
    
    snappy::RawCompress(static_cast<const char*>(uncomp_ptr), uncomp_len, static_cast<char*>(comp_ptr), &comp_size);
    
    return comp_size;
}

snappy::CompressedBuffer::CompressedBuffer() : comp_ptr(nullptr), comp_len(0)
{

}

snappy::CompressedBuffer::~CompressedBuffer()
{

}

snappy::CompressedBuffer::CompressedBuffer(const snappy::CompressedBuffer& other) : comp_ptr(other.comp_ptr), comp_len(other.comp_len)
{

}

snappy::CompressedBuffer::CompressedBuffer(snappy::CompressedBuffer&& other) : comp_ptr(other.comp_ptr), comp_len(other.comp_len)
{
    other.comp_ptr = 0;
}

snappy::CompressedBuffer& snappy::CompressedBuffer::operator=(const snappy::CompressedBuffer& other)
{
    comp_ptr = other.comp_ptr;
    comp_len = other.comp_len;
    return *this;
}

snappy::CompressedBuffer& snappy::CompressedBuffer::operator=(snappy::CompressedBuffer&& other)
{
    comp_ptr = other.comp_ptr;
    comp_len = other.comp_len;
    other.comp_ptr = 0;
    return *this;
}

snappy::CompressedBuffer::CompressedBuffer(const void* ptr, std::size_t s)
{
    reset(ptr, s);
}

void snappy::CompressedBuffer::reset(const void* ptr, std::size_t s)
{
    comp_ptr = ptr;
    comp_len = s;
}

bool snappy::CompressedBuffer::isValid() const
{
    if(comp_ptr != nullptr)
    {
        return snappy::IsValidCompressedBuffer(static_cast<const char*>(comp_ptr), comp_len);
    }
    else
    {
        return false;
    }
}

std::size_t snappy::CompressedBuffer::uncompressedSize() const
{
    std::size_t ret = 0;
    snappy::GetUncompressedLength(static_cast<const char*>(comp_ptr), comp_len, &ret);
    return ret;
}

bool snappy::CompressedBuffer::uncompress(void* outbuf)
{
    return snappy::RawUncompress(static_cast<const char*>(comp_ptr), comp_len, static_cast<char*>(outbuf));
}





