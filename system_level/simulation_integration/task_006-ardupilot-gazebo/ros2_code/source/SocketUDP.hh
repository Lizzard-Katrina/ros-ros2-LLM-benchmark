/*
   Copyright (C) 2024 ardupilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include <cstddef>

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#else
#include <winsock2.h>
#endif

class SocketUDP
{
public:
    SocketUDP(bool reuseaddress, bool blocking);
    ~SocketUDP();

    bool bind(const char *address, uint16_t port);
    bool set_reuseaddress();
    bool set_blocking(bool blocking);

    ssize_t sendto(const void *buf, size_t size, const char *address,
                   uint16_t port);
    ssize_t recv(void *buf, size_t size, uint32_t timeout_ms);

    void get_client_address(const char *&ip_addr, uint16_t &port);

    bool pollin(uint32_t timeout_ms);

private:
    void make_sockaddr(const char *address, uint16_t port,
                       struct sockaddr_in &sockaddr);

    int fd = -1;
    struct sockaddr_in in_addr{};
};