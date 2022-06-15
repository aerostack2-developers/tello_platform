#include "socketudp.hpp"

SocketUdp::SocketUdp(char *IP, int port, uint bufferSize){
    cout<<"Creating socket ..."<<endl;
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    this->IP = IP;
    this->port = port;
    buffer.resize(bufferSize, '\0');
}

SocketUdp::~SocketUdp(){
    cout<<"closing socket ..."<<endl;
    close(socket_fd);
}

void SocketUdp::configuration(){
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = inet_addr(IP);
    serv_addr.sin_family = AF_INET;
}


bool SocketUdp::setDest_addr(){
    bool success = true;
    addrinfo* addrInfo_{nullptr};
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    string port_str = to_string(port);
    int ret = getaddrinfo(IP, port_str.c_str(), &hints, &addrInfo_);
    if (ret != 0){
        cout<<"Error: setting dest_addr sockaddr_storage"<<endl;
        success = false;
    }
    memcpy(&dest_addr, addrInfo_->ai_addr, addrInfo_->ai_addrlen);
    freeaddrinfo(addrInfo_);
    return success;
}

bool SocketUdp::bindServer(){
    cout<<"Binding server ..."<<endl;
    bool connectionSucceded  = true;
    configuration();
    int connectionState = bind(socket_fd, reinterpret_cast<sockaddr*>(&serv_addr),
                      sizeof(serv_addr));
    if (connectionState == -1){
        cout<<"Unable to connect to IP: "<< IP <<" port: "<< port << endl;
        connectionSucceded = false;
    }
    bool setStorage = setDest_addr();
    
    if (!setStorage){
        cout<<"Unable to setDest_addr" << endl;
        connectionSucceded = false;
    }
    return connectionSucceded;
}


bool SocketUdp::sending(string message){
    bool sendOK = true;
    const vector<unsigned char> msgs{std::cbegin(message),
                                        std::cend(message)};
    const socklen_t dest_addr_len{sizeof(dest_addr)};

    int ret = sendto(socket_fd, msgs.data(), msgs.size(), 0,
                        reinterpret_cast<sockaddr*>(&dest_addr), dest_addr_len);
    if (ret ==-1){
        cout<<"sending: It has been impossible to send the message "<<message<< endl;
        sendOK = false;
    }
    return sendOK;
}
string SocketUdp::receiving(const int flags){
    string resp_res = "";
    socklen_t serv_addr_len{sizeof(dest_addr)};
    int ret = recvfrom(socket_fd, buffer.data(), buffer.size(), flags, 
        reinterpret_cast<sockaddr*>(&dest_addr), &serv_addr_len);

    if (ret == -1){
        return resp_res;
    }else if (ret < 1){
        return resp_res;
    }

    string resp{buffer.cbegin(), buffer.cbegin() + ret};
    resp.erase(resp.find_last_not_of(" \n\r\t") + 1);
    resp_res = resp;
    /*cout<<resp<<endl;
    cout<<resp_res<<endl;*/
    return resp_res;
}

int SocketUdp::getSocketfd (){
    return socket_fd;
}
char* SocketUdp::getIP(){
    return IP;
}

int SocketUdp::getPort(){
    return port;
}

void SocketUdp::setSocketfd(int socket_fd){
    this->socket_fd = socket_fd;
    
}

void SocketUdp::setIP(char* IP){
    this->IP = IP;
}

void SocketUdp::setPort(int port){
    this->port = port;
}
