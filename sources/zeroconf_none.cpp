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

#ifdef LOWLEVELCOMM_HAVE_DNSSD
#include <dns_sd.h>
#endif  // LOWLEVELCOMM_HAVE_DNSSD


zeroconf::ZeroConf::ZeroConf() : dns_service_ref_(0), stop_thread_(false) 
{
    
}

zeroconf::ZeroConf::~ZeroConf() 
{
    stop_thread_ = true;
    if(listen_to_server_thread_.joinable()) 
    {
        listen_to_server_thread_.join();
    }
}

bool zeroconf::ZeroConf::isValid()
{
    return false;
}

bool zeroconf::ZeroConf::registerService(const std::string& sName, const std::string& sRegType, uint16_t nPort, const std::string& sDomain) 
{
    return false;
}

std::vector<zeroconf::Record> zeroconf::ZeroConf::browseForServiceType(const std::string& sServiceType, const std::string& sDomain) 
{
    std::vector<zeroconf::Record> vRecords;
    return vRecords;
}

std::vector<zeroconf::URL> zeroconf::ZeroConf::resolveService(const std::string& sName, const std::string& sRegType, const std::string& sDomain, uint32_t nFlags, uint32_t nInterface) 
{
    return std::vector<zeroconf::URL>();
}

const char* zeroconf::ZeroConf::getHostIP() 
{
    char buf[128];
    gethostname(buf, sizeof(buf));
    struct hostent *hp = gethostbyname(buf);
    struct in_addr* in = (struct in_addr*)hp->h_addr;
    return inet_ntoa(*in);
}

void zeroconf::ZeroConf::resolveReplyCallback(DNSServiceRef, DNSServiceFlags nFlags, uint32_t, DNSServiceErrorType, 
                                              const char*, const char* hosttarget, uint16_t opaqueport, uint16_t, const unsigned char*) 
{

}

void zeroconf::ZeroConf::handleEventsThread() 
{

}

void zeroconf::ZeroConf::handleEventsInternal(DNSServiceRef ServiceRef) 
{

}
