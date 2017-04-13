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
 * Templated fixed bit width registers.
 * ****************************************************************************
 */
#ifndef MISCCPP_REGISTER_HPP
#define MISCCPP_REGISTER_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

/**
 * Direct memory access, static.
 * To be used on embedded systems.
 */
struct register_direct_access
{
    template<typename T>
    static inline T read(uintptr_t addr)
    {
        return *((const volatile T*)addr);
    }
    
    template<typename T>
    static inline void write(uintptr_t addr, T val)
    {
        *((volatile T*)addr) = val;
    }
};

/**
 * Static Single bit operator.
 */
template<std::size_t pos, typename DataType = volatile uint32_t>
class static_bit 
{
public:
    static inline std::size_t position()  { return pos; }
    
    template<typename T>
    static inline void clear(T& reg) 
    {
        reg.bitclear(position());
    }
    
    template<typename T>
    static inline void set(T& reg) 
    {
        reg.bitset(position());
    }
    
    template<typename T>
    static inline bool read(const T& reg) 
    {
        return reg.bittest(position());
    }
    
    template<typename T>
    static inline void write(T& reg, bool b) 
    {
        reg.bitwrite(position(), b);
    }
    
    static inline DataType mask() 
    {
        return (DataType)(1 << position());
    }
    
    static inline DataType bitsToSet(bool v)
    {
        if(v)
        {
            return mask();
        }
        else
        {
            return 0x00;
        }
    }
};
   
/**
 * Single bit operator.
 */
template<typename RegT>
class bit 
{
public:
    typedef RegT RegisterType;
    typedef typename RegisterType::DataType MaskType;
    
    inline bit(RegisterType& p, std::size_t b) noexcept : parent(p), b(b) { }
    
    inline std::size_t position() const noexcept { return b; }
    
    inline void clear() noexcept
    {
        const MaskType m = (MaskType)(1 << position());
        parent.maskclear(m);
    }
    
    inline void set() noexcept
    {
        const MaskType m = (MaskType)(1 << position());
        parent.maskset(m);
    }
    
    inline bool read() const noexcept
    {
        return parent.bittest(position());
    }
    
    inline void write(bool b) noexcept
    {
        b == true ? set() : clear();
    }
    
    explicit inline operator bool() const { return read(); }
    
    inline bit& operator=(bool d) noexcept
    {
        write(d);
        return *this;
    }
private:
    RegisterType& parent;
    std::size_t b;
};

/**
 * Static Typed bitfield operator.
 */
template<typename BT, std::size_t bitpos, std::size_t bitcount, typename DataType = volatile uint32_t>
class static_bitfield
{
public:
    typedef volatile BT BaseType;
    
    static constexpr inline std::size_t position() { return bitpos; }
    static constexpr inline std::size_t width() { return bitcount; }
    
    template<typename T>
    static inline BaseType read(const T& reg) 
    {
        return static_cast<BaseType>((reg.read() >> position()) & ~(~0 << (width())));
    }
    
    template<typename T>
    static inline void write(T& reg, BaseType v)
    {
        const DataType Mask = (((DataType)1 << width()) - 1) << position();
        reg.write((reg.read() & ~Mask) | ((static_cast<DataType>(v) << position()) & Mask));
    }
    
    static inline DataType mask() 
    {
        return (((DataType)1 << width()) - 1) << position();
    }
    
    static inline DataType bitsToSet(BaseType v)
    {
        const DataType Mask = (((DataType)1 << width()) - 1) << position();
        return ((static_cast<DataType>(v) << position()) & Mask);
    }
};

/**
 * Typed bitfield operator.
 */
template<typename bt, typename regT>
class bitfield
{
public:
    typedef bt BaseType;
    typedef regT RegisterType;
    typedef typename RegisterType::DataType MaskType;
    
    inline bitfield(RegisterType& p, std::size_t bp, std::size_t bc) : 
        parent(p), bitpos(bp), bitcount(bc)
    {
        
    }
    
    inline std::size_t position() const { return bitpos; }
    
    inline std::size_t count() const { return bitcount; }
    
    inline BaseType read() const
    {
        const MaskType Mask = (((MaskType)1 << count()) - 1) << position();
        return static_cast<BaseType>((parent.read() >> position()) & ~(~0 << (count())));
    }
    
    inline void write(BaseType v)
    {
        const MaskType Mask = (((MaskType)1 << count()) - 1) << position();
        parent.write((parent.read() & ~Mask) | ((static_cast<typename RegisterType::DataType>(v) << position()) & Mask));
    }
    
    inline operator BaseType() const { return read(); }
    
    inline bitfield& operator=(BaseType d)
    {
        write(d);
        return *this;
    }
    
private:
    RegisterType& parent;
    std::size_t bitpos, bitcount;
};

/**
 * Generic register operations.
 */
template<typename Derived, typename T>
class register_base
{
public:
    typedef T DataType;
    typedef T* AddressType;
    typedef register_base<Derived,T> RegisterType;
    
    static inline DataType bitmask(std::size_t bit)
    {
        return (DataType)(1 << bit);
    }
    
    inline bool bittest(std::size_t bit) const
    {
        return ( read() & bitmask(bit) ) != 0;
    }
    
    inline bool masktest(DataType mask) const
    {
        return mask == ( read() & mask );
    }
    
    inline void reset()
    {
        write(0);
    }
    
    inline void bitclear(std::size_t bit)
    {
        write( read() & ~bitmask( bit ) );
    }
    
    inline void bitset(std::size_t bit )
    {
        write( read() | bitmask( bit ) );
    }
    
    inline void bitwrite(std::size_t bit , bool v )
    {
        v == true ? bitset(bit) : bitclear(bit);
    }
    
    inline void maskclear(DataType mask)
    {
        write( read() & ~mask );
    }
    
    inline void maskset( DataType mask )
    {
        write( read() | mask );
    }
    
    inline void maskset( DataType clearmask, DataType setmask )
    {
        const DataType val = read();
        write((val & ~clearmask) | (val & setmask));
    }
    
    template<typename OutputType>
    inline OutputType read_field(std::size_t bitpos, std::size_t bitcnt)
    {
        const DataType Mask = (((DataType)1 << bitcnt) - 1) << bitpos;
        return static_cast<OutputType>((read() >> bitpos) & ~(~0 << (bitcnt)));
    }
    
    template<typename OutputType>
    inline void write_field(std::size_t bitpos, std::size_t bitcnt, const OutputType& val)
    {
        const DataType Mask = (((DataType)1 << bitcnt) - 1) << bitpos;
        write((read() & ~Mask) | ((static_cast<DataType>(val) << bitpos) & Mask));
    }
    
    inline DataType read() const
    {
        return ((const Derived*)this)->read();
    }
    
    inline void write(DataType value)
    {
        ((Derived*)this)->write(value);
    }
    
    inline operator DataType() const { return read(); }
    
    inline register_base& operator=(DataType d)
    {
        write(d);
        return *this;
    }
    
    inline register_base& operator|=(DataType d)
    {
        write(read() | d);
        return *this;
    }
    
    inline register_base& operator&=(DataType d)
    {
        write(read() & d);
        return *this;
    }
};

/**
 * Fixed location register.
 */
template<typename T>
class fixed_register : public register_base<fixed_register<T>, volatile T>
{
public:
    typedef volatile T DataType;
    typedef uintptr_t AddressType;
    typedef register_base<fixed_register<T>, volatile T> BaseType;
    typedef fixed_register<T> RegisterType;
    typedef std::false_type AccessType;
    static constexpr bool StaticAccess = true;
    static constexpr bool HardcodedAddress = true;
    
    inline AddressType address() const { return static_cast<AddressType>(this); }
    
    using BaseType::operator DataType;
    using BaseType::operator=;
    using BaseType::operator|=;
    using BaseType::operator&=;

    inline DataType read() const
    {
        return _value;
    }
    
    inline void write(DataType value)
    {
        _value = value;
    }
    
private:
    DataType _value;
};

/**
 * Fixed location register with external access.
 */
template<typename T, uintptr_t Offset, typename Access = register_direct_access>
class fixed_register_access : public register_base<fixed_register_access<T,Offset,Access>,T>
{
public:
    typedef T DataType;
    typedef uintptr_t AddressType;
    typedef register_base<fixed_register_access<T,Offset,Access>,T> BaseType;
    typedef fixed_register_access<T,Offset,Access> RegisterType;
    typedef Access AccessType;
    static constexpr bool StaticAccess = true;
    static constexpr bool HardcodedAddress = true;
    
    inline AddressType address() const { return static_cast<AddressType>(Offset); }
    
    using BaseType::operator DataType;
    using BaseType::operator=;
    using BaseType::operator|=;
    using BaseType::operator&=;
private:
    inline DataType read() const
    {
        return Access::template read<T>(address());
    }
    
    inline void write(DataType value)
    {
        Access::template write<T>(address(), value);
    }
    
    friend class register_base<fixed_register_access<T,Offset,Access>,T>;
    template<std::size_t pos, typename DataType> friend class static_bit;
    template<typename BT, std::size_t bitpos, std::size_t bitcount, typename DataType> friend class static_bitfield;
};

/**
 * Dynamic address register with dynamic memory access.
 */
template<typename T, typename Access = register_direct_access>
class dynamic_access_dynamic_register : public register_base<dynamic_access_dynamic_register<T,Access>,T>
{
public:
    typedef T DataType;
    typedef uintptr_t AddressType;
    typedef register_base<dynamic_access_dynamic_register<T,Access>,T> BaseType;
    typedef dynamic_access_dynamic_register<T,Access> RegisterType;
    typedef Access AccessType;
    static constexpr bool StaticAccess = false;
    static constexpr bool HardcodedAddress = false;
    
    inline dynamic_access_dynamic_register(AddressType aaddr, AccessType& a) : addr(aaddr), acc(a)
    { 
        
    }
    
    inline DataType read() const
    {
        return acc.template read<T>(address());
    }
    
    inline void write(DataType value)
    {
        acc.template write<T>(address(), value);
    }
    
    inline AddressType address() const { return addr; }
    
    using BaseType::operator DataType;
    using BaseType::operator=;
    using BaseType::operator|=;
    using BaseType::operator&=;
private:
    AddressType addr;
    AccessType& acc;
};

/**
 * Fixed address register with static memory access.
 */
template<typename T, uintptr_t ADDR, typename Access = register_direct_access>
class static_access_static_register : public register_base<static_access_static_register<T,ADDR,Access>,T>
{
public:    
    typedef T DataType;
    typedef uintptr_t AddressType;
    typedef register_base<static_access_static_register<T,ADDR,Access>,T> BaseType;
    typedef static_access_static_register<T,ADDR,Access> RegisterType;
    typedef Access AccessType;
    static constexpr bool StaticAccess = true;
    static constexpr bool HardcodedAddress = true;
    
    inline static_access_static_register() { }
    
    inline DataType read() const
    {
        return Access::template read<T>(address());
    }
    
    inline void write(DataType value)
    {
        Access::template write<T>(address(), value);
    }
    
    static inline AddressType address() { return ADDR; }
    
    using BaseType::operator DataType;
    using BaseType::operator=;
    using BaseType::operator|=;
    using BaseType::operator&=;
};

/**
 * Fixed address register with dynamic memory access.
 */
template<typename T, uintptr_t ADDR, typename Access = register_direct_access>
class dynamic_access_static_register : public register_base<dynamic_access_static_register<T,ADDR,Access>,T>
{
public:    
    typedef T DataType;
    typedef uintptr_t AddressType;
    typedef register_base<dynamic_access_static_register<T,ADDR,Access>,T> BaseType;
    typedef dynamic_access_static_register<T,ADDR,Access> RegisterType;
    typedef Access AccessType;
    static constexpr bool StaticAccess = false;
    static constexpr bool HardcodedAddress = true;
    
    inline dynamic_access_static_register(AccessType& a) : acc(a) { }
    
    inline DataType read() const
    {
        return acc.template read<T>(address());
    }
    
    inline void write(DataType value)
    {
        acc.template write<T>(address(), value);
    }
    
    static inline AddressType address() { return ADDR; }
    
    using BaseType::operator DataType;
    using BaseType::operator=;
    using BaseType::operator|=;
    using BaseType::operator&=;
    
private:
    AccessType& acc;
};

/**
 * Dynamic address register with static memory access.
 */
template<typename T, typename Access = register_direct_access>
class static_access_dynamic_register : public register_base<static_access_dynamic_register<T,Access>,T>
{
public:
    typedef T DataType;
    typedef uintptr_t AddressType;
    typedef register_base<static_access_dynamic_register<T,Access>,T> BaseType;
    typedef static_access_dynamic_register<T,Access> RegisterType;
    typedef Access AccessType;
    static constexpr bool StaticAccess = true;
    static constexpr bool HardcodedAddress = false;
    
    inline static_access_dynamic_register(AddressType aaddr) : addr(aaddr)
    { 
        
    }
    
    inline DataType read() const
    {
        return Access::template read<T>(addr);
    }
    
    inline void write(DataType value)
    {
        Access::template write<T>(addr, value);
    }
    
    inline AddressType address() const { return addr; }
    
    using BaseType::operator DataType;
    using BaseType::operator=;
    using BaseType::operator|=;
    using BaseType::operator&=;
private:
    AddressType addr;
};

/**
 * Fixed address register with static memory access.
 */
template<typename T>
class fixed_memory_register : public register_base<fixed_memory_register<T>,T>
{
public:    
    typedef T DataType;
    typedef uintptr_t AddressType;
    typedef register_base<fixed_memory_register<T>,T> BaseType;
    typedef fixed_memory_register<T> RegisterType;
    typedef register_direct_access AccessType;
    static constexpr bool StaticAccess = true;
    static constexpr bool HardcodedAddress = true;
    
    inline fixed_memory_register(DataType* addr) : variable(addr) { }
    
    inline DataType read() const
    {
        return *variable;
    }
    
    inline void write(DataType value)
    {
        *variable = value;
    }
    
    inline AddressType address() { return (AddressType)variable; }
    
    using BaseType::operator DataType;
    using BaseType::operator=;
    using BaseType::operator|=;
    using BaseType::operator&=;
private:
    volatile DataType* variable;
};

#endif // MISCCPP_REGISTER_HPP
