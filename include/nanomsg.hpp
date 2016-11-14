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
 * NanoMsg thin C++11 wrapper evolved from cppnanomsg.
 * ****************************************************************************
 */

#ifndef NANOMSG_HPP
#define NANOMSG_HPP

#include <nanomsg/nn.h>

#include <nanomsg/reqrep.h>
#include <nanomsg/bus.h>
#include <nanomsg/pair.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/pipeline.h>
#include <nanomsg/survey.h>

#include <nanomsg/inproc.h>
#include <nanomsg/ipc.h>
#include <nanomsg/tcp.h>

#include <cstddef>
#include <algorithm>
#include <cassert>
#include <exception>

#if defined __GNUC__
#define nn_slow(x) __builtin_expect ((x), 0)
#else
#define nn_slow(x) (x)
#endif

namespace nn
{
    
class exception : public ::std::exception
{
public:
    exception() : error_number(nn_errno()) { }
    virtual ~exception() { }
    virtual const char* what() const noexcept { return nn_strerror(error_number); }
    inline int error_code() const { return error_number; }
private:
    int error_number;
};

typedef int endpoint_id;

class socket_t;

class message_t
{
public:
    // default constructor
    inline message_t() : buf(0), len(0) { }

    // allocation constructor
    inline explicit message_t(::std::size_t l, int type = 0) : buf(0), len(l)
    {
        buf = nn_allocmsg(len, type);
        if(buf == 0) { throw exception(); }
    }
    
    // move constructor
    inline message_t(message_t&& m) noexcept : buf(m.buf), len(m.len) { m.buf = 0; m.len = 0; }
    
    // move assignment
    inline message_t& operator=(message_t&& m) noexcept
    {
        m.swap(*this);
        return *this;
    }
    
    // non-copyable
    message_t(const message_t&) = delete;
    message_t& operator=(const message_t&) = delete;
    
    // destructor
    inline ~message_t() { if(buf != 0) { nn_freemsg(buf); } }
    
    // access
    inline const void* data() const { return buf; }
    inline void* data() { return buf; }
    inline ::std::size_t size() const { return len; }
    
    // has data?
    inline operator bool () const noexcept { return buf != 0; }
    
    inline void swap(message_t &m) noexcept
    {
        ::std::swap(buf, m.buf);
        ::std::swap(len, m.len);
    }
private:
    friend class socket_t;
    mutable void* buf;
    mutable ::std::size_t len;
};

static void poll(nn_pollfd *fds, int nfds, int timeout);
static void device(const socket_t& s1, const socket_t& s2);
    
class socket_t
{    
public:
    enum class statistic : int
    {
        established_connections = NN_STAT_ESTABLISHED_CONNECTIONS,
        accepted_connections = NN_STAT_ACCEPTED_CONNECTIONS,
        dropped_connections = NN_STAT_DROPPED_CONNECTIONS,
        broken_connections = NN_STAT_BROKEN_CONNECTIONS,
        connect_errors = NN_STAT_CONNECT_ERRORS,
        bind_errors = NN_STAT_BIND_ERRORS,
        accept_errors = NN_STAT_ACCEPT_ERRORS,
        current_connections_count = NN_STAT_CURRENT_CONNECTIONS,
        inprogress_connections_count = NN_STAT_INPROGRESS_CONNECTIONS,
        current_ep_error_count = NN_STAT_CURRENT_EP_ERRORS,
        messages_sent = NN_STAT_MESSAGES_SENT,
        messages_received = NN_STAT_MESSAGES_RECEIVED,
        bytes_sent = NN_STAT_BYTES_SENT,
        bytes_received = NN_STAT_BYTES_RECEIVED,
        current_snd_priority = NN_STAT_CURRENT_SND_PRIORITY
    };
    
    // default constructor
    inline socket_t() : sock(-1) { }
    
    // common constructor
    inline explicit socket_t(int proto, int domain = AF_SP) : sock(-1)
    {
        sock = nn_socket(domain, proto);
        if(nn_slow(sock < 0)) { throw exception(); }
    }
    
    // move constructor
    inline socket_t(socket_t&& s) noexcept : sock(s.sock) { s.sock = -1; }
    
    // move assignment
    inline socket_t& operator=(socket_t&& s) noexcept
    {
        s.swap(*this);
        return *this;
    }
    
    // non-copyable
    socket_t(const socket_t&) = delete;
    socket_t& operator=(const socket_t&) = delete;
    
    // destructor
    inline ~socket_t() 
    { 
        close();
    }
    
    inline void close()
    {
        if(sock != 0)
        {
            int ret = nn_close(sock); 
            assert(ret == 0);
            (void)ret;
            sock = 0;
        }
    }
    
    inline operator bool () const noexcept { return sock >= 0; }
    
    inline void swap(socket_t &s) noexcept { ::std::swap(sock, s.sock); }
    
    template<typename T>
    inline void getsockopt(int level, int option, T& val)
    {
        ::std::size_t len = sizeof(T);
        getsockopt(level, option, &val, &len);
    }
    
    inline void getsockopt(int level, int option, void* optval, ::std::size_t* vallen)
    {
        int ret = nn_getsockopt(sock, level, option, optval, vallen);
        if(nn_slow(ret < 0)) { throw exception(); }
    }
    
    template<typename T>
    inline void setsockopt(int level, int option, const T& val)
    {
        setsockopt(level, option, &val, sizeof(T));
    }
    
    inline void setsockopt(int level, int option, const void* val, ::std::size_t vallen)
    {
        int ret = nn_setsockopt(sock, level, option, val, vallen);
        if(nn_slow(ret < 0)) { throw exception(); }
    }
    
    inline uint64_t get_statistic(statistic s)
    {
        return nn_get_statistic(sock, (int)s);
    }
    
    inline endpoint_id bind(const char* addr)
    {
        endpoint_id ret = nn_bind(sock, addr);
        if(nn_slow(ret < 0)) { throw exception(); }
        return ret;
    }
    
    inline endpoint_id connect(const char* addr)
    {
        endpoint_id ret = nn_connect(sock, addr);
        if(nn_slow(ret < 0)) { throw exception(); }
        return ret;
    }
    
    inline void shutdown(endpoint_id eid)
    {
        int ret = nn_shutdown(sock, eid);
        if(nn_slow(ret < 0)) { throw exception(); }
    }
    
    inline void send(const void* buffer, ::std::size_t len, bool dontwait = false)
    {
        int ret = nn_send(sock, buffer, len, dontwait == true ? NN_DONTWAIT : 0);
        if(nn_slow(ret < 0)) { throw exception(); }
    }
    
    inline void receive(void* buffer, ::std::size_t len, bool dontwait = false)
    {
        int ret = nn_recv(sock, buffer, len, dontwait == true ? NN_DONTWAIT : 0);
        if(nn_slow(ret < 0)) { throw exception(); }
    }
    
    inline void send(const message_t& msg, bool dontwait = false)
    {
        int ret = nn_send(sock, &(msg.buf), NN_MSG, dontwait == true ? NN_DONTWAIT : 0);
        if(nn_slow(ret < 0)) { throw exception(); } else { msg.buf = 0; msg.len = 0; } // NOTE send frees message
    }
    
    inline void receive(message_t& msg, bool dontwait = false)
    {
        int ret = nn_recv(sock, &(msg.buf), NN_MSG, dontwait == true ? NN_DONTWAIT : 0);
        if(nn_slow(ret < 0)) { throw exception(); } else { msg.len = ret; }
    }
    
    inline void setFD(nn_pollfd& fd)
    {
        fd.fd = sock;
    }
private:
    friend void device(const socket_t& s1, const socket_t& s2);
    
    int sock;
};

static void poll(nn_pollfd *fds, int nfds, int timeout) __attribute__ ((unused));

static void poll(nn_pollfd *fds, int nfds, int timeout)
{
    int ret = nn_poll(fds, nfds, timeout);
    if(nn_slow(ret < 0)) { throw exception(); }
}

static void device(const socket_t& s1, const socket_t& s2) __attribute__ ((unused));
static void device(const socket_t& s1, const socket_t& s2)
{
    int ret = nn_device(s1.sock, s2.sock);
    if(nn_slow(ret < 0)) { throw exception(); }
}

static void terminate() __attribute__ ((unused));
static void terminate() { nn_term(); } 
    
}

#endif // NANOMSG_HPP
