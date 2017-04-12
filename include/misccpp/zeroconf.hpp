/**
 * 
 * Low-Level Communications.
 * ZeroConf Client.
 * 
 * Copyright (c) Robert Lukierski 2015. All rights reserved.
 * Author: Robert Lukierski.
 * 
 */
#ifndef ZEROCONF_HPP
#define ZEROCONF_HPP

#include <iostream>
#include <string>
#include <memory>

class ZeroConf
{
    struct Pimpl;
public:
    static ZeroConf& getInstance()
    {
        static ZeroConf zc;
        return zc;
    }
    
    bool registerService(const std::string& sName, const std::string& sRegType, uint16_t nPort, const std::string& sDomain = "local");
    bool resolveService(const std::string& sName, const std::string& sRegType, std::string& host, uint16_t& nPort, const std::string& sDomain = "local");
private:
    ZeroConf();
    ~ZeroConf();
    std::unique_ptr<Pimpl> pimpl;
};

#endif // ZEROCONF_HPP