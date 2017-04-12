#include <inttypes.h>
#include <errno.h>
#include <sys/types.h> // for u_char
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdlib.h>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <glog/logging.h>

#include <zeroconf.hpp>

#include <condition_variable>
#include <mutex>

#include <avahi-common/thread-watch.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/error.h>
#include <avahi-common/address.h>
#include <avahi-common/domain.h>
#include <avahi-client/lookup.h>
#include <avahi-client/publish.h>

struct ZeroConf::Pimpl
{
    static void cb_client(AvahiClient *c, AvahiClientState state, AVAHI_GCC_UNUSED void * userdata) 
    {
        Pimpl* pp = static_cast<Pimpl*>(userdata);
        
        switch (state)
        {
            case AVAHI_CLIENT_S_RUNNING:
                pp->spin();
                break;
            case AVAHI_CLIENT_FAILURE:
                pp->fail();
                break;
            case AVAHI_CLIENT_S_COLLISION:
                break;
            case AVAHI_CLIENT_S_REGISTERING:
                break;
            case AVAHI_CLIENT_CONNECTING:
                break;
        }
    }
    
    static void cb_group(AvahiEntryGroup *g, AvahiEntryGroupState state, void* userdata)
    {
        #if 0
        switch(state)
        {
            case AVAHI_ENTRY_GROUP_UNCOMMITED:  LOG(INFO) << "GroupCB: AVAHI_ENTRY_GROUP_UNCOMMITED"; break;
            case AVAHI_ENTRY_GROUP_REGISTERING: LOG(INFO) << "GroupCB: AVAHI_ENTRY_GROUP_REGISTERING"; break;
            case AVAHI_ENTRY_GROUP_ESTABLISHED: LOG(INFO) << "GroupCB: AVAHI_ENTRY_GROUP_ESTABLISHED"; break;
            case AVAHI_ENTRY_GROUP_COLLISION: LOG(INFO) << "GroupCB: AVAHI_ENTRY_GROUP_COLLISION"; break;
            case AVAHI_ENTRY_GROUP_FAILURE: LOG(INFO) << "GroupCB: AVAHI_ENTRY_GROUP_FAILURE"; break;
    }
    #endif
    }
    
    Pimpl() : invalid_object(false), threaded_poll(nullptr), client(nullptr)
    {
        if(!(threaded_poll = avahi_threaded_poll_new()))
        {
            LOG(ERROR) << "Zeroconf: failed to create an avahi threaded  poll.";
            invalid_object = true;
            return;
        }
        
        int error;
        
        client = avahi_client_new(avahi_threaded_poll_get(threaded_poll), static_cast<AvahiClientFlags>(0), Pimpl::cb_client, this, &error);
        
        if(!client)
        {
            LOG(ERROR) << "Zeroconf: failed to create an avahi client.";
            invalid_object = true;
            return;
        }
    }
    
    ~Pimpl()
    {
        for(auto& grp : groups)
        {
            avahi_entry_group_reset(grp);
            avahi_entry_group_free(grp);
        }
        
        if(threaded_poll)
        {
            avahi_threaded_poll_stop(threaded_poll);
        }
        
        if(client)
        {
            avahi_client_free(client);
        }
        
        if(threaded_poll)
        {
            avahi_threaded_poll_free(threaded_poll);
        }
    }
    
    void spin()
    {
        if(!invalid_object)
        {
            avahi_threaded_poll_start(threaded_poll);
        }
    }
    
    void fail()
    {
        avahi_threaded_poll_quit(threaded_poll);
        invalid_object = true;
    }
    
    bool is_alive() const { return !invalid_object; }
    
    bool registerServiceThreaded(const std::string& sName, const std::string& sRegType, uint16_t nPort, const std::string& sDomain = "local")
    {
        AvahiEntryGroup* entgrp = avahi_entry_group_new(client, cb_group, this);
        
        int ret = avahi_entry_group_add_service(entgrp, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC, static_cast<AvahiPublishFlags>(0), sName.c_str(), sRegType.c_str(), sDomain.c_str(), NULL, nPort, NULL);
        
        bool retf = false;
        
        if(ret == AVAHI_OK)
        {
            int ret2 = avahi_entry_group_commit(entgrp);
            if(ret2 == AVAHI_OK)
            {
                std::lock_guard<std::mutex> l(mut_grps);
                groups.push_back(entgrp);
                retf = true;
            }
            else
            {
                avahi_entry_group_free(entgrp);
            }
        }
        else
        {
            avahi_entry_group_free(entgrp);
        }
        
        
        return retf;
    }
    
    bool registerService(const std::string& sName, const std::string& sRegType, uint16_t nPort, const std::string& sDomain = "local")
    {
        avahi_threaded_poll_lock(threaded_poll);
        bool result = registerServiceThreaded(sName, sRegType, nPort, sDomain);
        avahi_threaded_poll_unlock(threaded_poll);
        return result;
    }
    
    static void cb_resolve(AvahiServiceResolver *r, AvahiIfIndex interface, AvahiProtocol protocol, AvahiResolverEvent event, const char *name, const char *type, const char *domain, const char *host_name, const AvahiAddress *a, uint16_t port, AvahiStringList *txt, AvahiLookupResultFlags flags, void *userdata)
    {
        Pimpl* pp = static_cast<Pimpl*>(userdata);
        
        pp->got_found = false;
        
        if(event == AVAHI_RESOLVER_FAILURE)
        {
            pp->got_found = false;
        }
        else if(event == AVAHI_RESOLVER_FOUND)
        {
            pp->got_found = true;
            pp->got_port = port;
            pp->got_hostname = std::string(host_name);
        }
        
        {
            std::lock_guard<std::mutex> lk(pp->mut);
            pp->got_answer = true;
        }
        
        pp->cv.notify_one();
    }
    
    bool resolveServiceThreaded(const std::string& sName, const std::string& sRegType, std::string& host, uint16_t& nPort, const std::string& sDomain = "local")
    {
        got_answer = false;
        
        avahi_threaded_poll_lock(threaded_poll);
        AvahiServiceResolver* sres = avahi_service_resolver_new(client, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC, sName.c_str(),  sRegType.c_str(), sDomain.c_str(), AVAHI_PROTO_UNSPEC, static_cast<AvahiLookupFlags>(0), Pimpl::cb_resolve, this);
        avahi_threaded_poll_unlock(threaded_poll);
        
        if(sres)
        {
            {
                std::unique_lock<std::mutex> lk(mut);
                bool ok = cv.wait_for(lk, std::chrono::milliseconds(1000), [&]{return got_answer;});
                if(ok == false)
                {
                    avahi_service_resolver_free(sres);
                    return false;
                }
            }
            
            bool ret = got_found;
            host = got_hostname;
            nPort = got_port;
            
            avahi_service_resolver_free(sres);
            
            return ret;
        }
        else
        {
            return false;
        }
    }
    
    bool resolveService(const std::string& sName, const std::string& sRegType, std::string& host, uint16_t& nPort, const std::string& sDomain = "local")
    {
        //avahi_threaded_poll_lock(threaded_poll);
        bool result = resolveServiceThreaded(sName, sRegType, host, nPort, sDomain);
        //avahi_threaded_poll_unlock(threaded_poll);
        return result;
    }
    
    bool invalid_object;
    AvahiThreadedPoll* threaded_poll;
    AvahiClient* client;
    std::mutex mut;
    std::condition_variable cv;
    bool got_answer;
    bool got_found;
    std::string got_hostname;
    uint16_t got_port;
    std::mutex mut_grps;
    std::vector<AvahiEntryGroup*> groups;
};

ZeroConf::ZeroConf() : pimpl(new Pimpl())
{
    
}

ZeroConf::~ZeroConf()
{
    
}

bool ZeroConf::registerService(const std::string& sName, const std::string& sRegType, uint16_t nPort, const std::string& sDomain)
{
    return pimpl->registerService(sName, sRegType, nPort, sDomain);
}

bool ZeroConf::resolveService(const std::string& sName, const std::string& sRegType, std::string& host, uint16_t& nPort, const std::string& sDomain)
{
    return pimpl->resolveService(sName, sRegType, host, nPort, sDomain);
}