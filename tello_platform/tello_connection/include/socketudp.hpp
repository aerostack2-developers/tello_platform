#ifndef SOCKETUDP_H
#define SOCKETUDP_H

#include <iostream>
#include <stdio.h>

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <vector>
#include <sstream>
using namespace std;

class SocketUdp {
    private:
        int socket_fd;
        char* IP;
        int port;
        sockaddr_in serv_addr;
        vector<unsigned char> buffer;
        sockaddr_storage dest_addr;

    private:
        void configuration();
        bool setDest_addr();
    public:
        SocketUdp (char* IP = "0.0.0.0", int port = 0, uint bufferSize=1024);
        ~SocketUdp(); // closisng socket

        bool bindServer();
        int getSocketfd ();
        char* getIP();
        int getPort();

        void setSocketfd (int socket_fd);
        void setIP (char* IP);
        void setPort (int port);

        bool sending(string message);
        string receiving(const int flags=MSG_DONTWAIT);
};

#endif //SOCKETUDP_H