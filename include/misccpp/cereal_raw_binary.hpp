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
 * Tightly packed cereal binary archive, not portable at all!
 * ****************************************************************************
 */

#ifndef CEREAL_ARCHIVES_RAW_BINARY_HPP_
#define CEREAL_ARCHIVES_RAW_BINARY_HPP_

#include <cereal/cereal.hpp>
#include <cstring>

namespace cereal
{

class RawByteCountingArchive : public cereal::OutputArchive<RawByteCountingArchive, cereal::AllowEmptyClassElision>
{
public:
    RawByteCountingArchive() : cereal::OutputArchive<RawByteCountingArchive, cereal::AllowEmptyClassElision>(this), current_size(0) { }
    
    //! Writes size bytes of data to the output stream
    void saveBinary( const void * data, std::size_t size) { current_size += size; }
    
    void reset() { current_size = 0; }
    inline std::size_t totalBytes() const { return current_size; }
    inline void addExtra(std::size_t bytes) { current_size += bytes; }
private:
    std::size_t current_size;
};
    
class RawBinaryOutputArchive : public OutputArchive<RawBinaryOutputArchive, AllowEmptyClassElision>
{
public:
    RawBinaryOutputArchive(void* ptr, std::size_t ms) :
        OutputArchive<RawBinaryOutputArchive, AllowEmptyClassElision>(this),
        output(static_cast<uint8_t*>(ptr)),
        maximum_size(ms),
        current_size(0),
        got_error(false)
    {
        
    }

    //! Writes size bytes of data to the output stream
    void saveBinary( const void * data, std::size_t size )
    {
        if(current_size + size <= maximum_size)
        {
            memcpy(&output[current_size], data, size);
            current_size += size;
        }
        else
        {
            got_error = true; // too much
        }
    }

    inline bool isError() const { return got_error; }
    void reset() { current_size = 0; got_error = false; }
    inline std::size_t maximumBytes() const { return maximum_size; }
    inline std::size_t bytesWritten() const { return current_size; }
    inline void skip(std::size_t bytes) { current_size += bytes; }
private:
    uint8_t* output;
    std::size_t maximum_size;
    std::size_t current_size;
    bool got_error;
};

class RawBinaryInputArchive : public InputArchive<RawBinaryInputArchive, AllowEmptyClassElision>
{
public:
    RawBinaryInputArchive(const void* ptr, std::size_t ms) :
        InputArchive<RawBinaryInputArchive, AllowEmptyClassElision>(this),
        input(static_cast<const uint8_t*>(ptr)),
        maximum_size(ms),
        current_size(0),
        got_error(false)
    { 
        
    }

    //! Reads size bytes of data from the input stream
    void loadBinary( void * const data, std::size_t size )
    {
        if(current_size + size <= maximum_size)
        {
            memcpy(data, &input[current_size], size);
            current_size += size;
        }
        else
        {
            got_error = true; // not enough data provided
        }
    }

    inline bool isError() const { return got_error; }
    void reset() { current_size = 0; got_error = false; }
    inline std::size_t maximumBytes() const { return maximum_size; }
    inline std::size_t bytesRead() const { return current_size; }
    inline void skip(std::size_t bytes) { current_size += bytes; }
private:
    const uint8_t* input;
    std::size_t maximum_size;
    std::size_t current_size;
    bool got_error;
};

// ######################################################################
// Common RawBinaryArchive serialization functions

//! Saving for POD types to binary
template<class T> inline
typename std::enable_if<std::is_arithmetic<T>::value, void>::type
save(RawBinaryOutputArchive & ar, T const & t)
{
    ar.saveBinary(std::addressof(t), sizeof(t));
}

//! Loading for POD types from binary
template<class T> inline
typename std::enable_if<std::is_arithmetic<T>::value, void>::type
load(RawBinaryInputArchive & ar, T & t)
{
    ar.loadBinary(std::addressof(t), sizeof(t));
}

//! Serializing NVP types to binary
template <class Archive, class T> inline
CEREAL_ARCHIVE_RESTRICT(RawBinaryInputArchive, RawBinaryOutputArchive)
serialize( Archive & ar, NameValuePair<T> & t )
{
    ar( t.value );
}

//! Serializing SizeTags to binary
template <class Archive, class T> inline
CEREAL_ARCHIVE_RESTRICT(RawBinaryInputArchive, RawBinaryOutputArchive)
serialize( Archive & ar, SizeTag<T> & t )
{
    ar( t.size );
}

//! Loading binary data
template <class T> inline
void load(RawBinaryInputArchive & ar, BinaryData<T> & bd)
{
    ar.loadBinary(bd.data, static_cast<std::size_t>(bd.size));
}

//! Saving binary data
template <class T> inline
void save(RawBinaryOutputArchive & ar, BinaryData<T> const & bd)
{
    ar.saveBinary( bd.data, static_cast<std::size_t>( bd.size ) );
}
#if 0 
TODO FIXME
//! Loading C-string
inline void load(RawBinaryInputArchive & ar, CStringData & bd)
{
    size_type len = 0;
    
    // read length
    ar(len);
    
    if(len > bd.len) { len = bd.len; }
    
    if(len > 0)
    {
        ar.loadBinary(bd.data, static_cast<std::size_t>(len)); // read bytes
        bd.data[len] = 0; // terminator
    }
}

//! Saving C-string
inline void save(RawBinaryOutputArchive & ar, CStringData const & bd)
{
    size_type len = 0;
    
    if(bd.data != 0)
    {
        len = strlen(static_cast<const char*>(bd.data));
    }
    
    if(len > bd.len) { len = bd.len; } // limit
    
    // save length
    ar(len);
    
    if(len > 0)
    {
        ar.saveBinary(bd.data, static_cast<std::size_t>( len ) );
    }
}
#endif
/// --------------------------------------------------------------------

// Counting NameValuePair
template <class T> inline
void serialize( RawByteCountingArchive & ar, NameValuePair<T> & t )
{
    ar( t.value );
}

// Counting SizeTag
template <class T> inline
void serialize( RawByteCountingArchive & ar, SizeTag<T> & t )
{
    ar( t.size );
}

// Counting POD data
template<class T> inline
typename std::enable_if<std::is_arithmetic<T>::value, void>::type
save(RawByteCountingArchive & ar, T const & t)
{
    ar.saveBinary(std::addressof(t), sizeof(t));
}

// Counting BinaryData
template <class T> inline
void save(RawByteCountingArchive & ar, BinaryData<T> const & bd)
{
    ar.saveBinary( bd.data, static_cast<std::size_t>( bd.size ) );
}
#if 0
TODO FIXME
//! Counting C-string
inline void save(RawByteCountingArchive & ar, CStringData const & bd)
{
    size_type len = 0;
    
    if(bd.data != 0)
    {
        len = strlen(static_cast<const char*>(bd.data));
    }
    
    if(len > bd.len) { len = bd.len; } // limit
    
    // save length
    ar(len);
    
    if(len > 0)
    {
        ar.saveBinary(bd.data, static_cast<std::size_t>( len ) );
    }
}
#endif
} // namespace cereal

// register archives for polymorphic support
CEREAL_REGISTER_ARCHIVE(cereal::RawByteCountingArchive)
CEREAL_REGISTER_ARCHIVE(cereal::RawBinaryOutputArchive)
CEREAL_REGISTER_ARCHIVE(cereal::RawBinaryInputArchive)

// tie input and output archives together
CEREAL_SETUP_ARCHIVE_TRAITS(cereal::RawBinaryInputArchive, cereal::RawBinaryOutputArchive)

#endif // CEREAL_ARCHIVES_RAW_BINARY_HPP_
