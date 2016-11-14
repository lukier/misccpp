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

#ifndef SNAPPY_HPP
#define SNAPPY_HPP

namespace snappy
{
    
class CompressedBuffer
{
public:
    CompressedBuffer();
    virtual ~CompressedBuffer();
    
    CompressedBuffer(const CompressedBuffer& other);
    CompressedBuffer(CompressedBuffer&& other);
    CompressedBuffer(const void* ptr, std::size_t s);
    
    CompressedBuffer& operator=(const CompressedBuffer& other);
    CompressedBuffer& operator=(CompressedBuffer&& other);
    
    void reset(const void* ptr, std::size_t s);
    bool isValid() const;
    
    const void* compressedData() const { return comp_ptr; }
    std::size_t compressedSize() const { return comp_len; }
    std::size_t uncompressedSize() const;
    
    bool uncompress(void* outbuf);
private:
    const void* comp_ptr;
    std::size_t comp_len;
};

std::size_t maxCompressedSize(std::size_t uncomp_len);
std::size_t compress(const void* uncomp_ptr, std::size_t uncomp_len, void* comp_ptr);
    
}

#endif // SNAPPY_HPP
