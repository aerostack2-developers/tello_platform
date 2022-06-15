#include <iostream>
#include <sys/socket.h>
#include "tello.hpp"
#include <arpa/inet.h>

 
Tello::Tello(){
    //cout<<"Creating sockets ..."<<endl;
    commandSender = new SocketUdp;
    stateRecv = new SocketUdp;

    commandSender->setIP((char*)IP_command);
    commandSender->setPort(port_command);

    stateRecv->setPort(port_state);

    commandSender->bindServer();

    pair<bool, string> response = sendCommand("command");
    if (response.first == false or response.second=="Error"){
        cout<<"Error: Connecting to tello"<< endl; 
        exit(-1);
    }
    stateRecv->bindServer();
    state.resize(16);
    update();
    
    
    thread stateThd(&Tello::threadStateFnc, this);
    stateThd.detach();
    thread videoThd(&Tello::streamVideo, this);
    videoThd.detach();
    
}


Tello::~Tello(){
    commandSender->~SocketUdp();
    stateRecv->~SocketUdp();

    delete(commandSender);
    delete(stateRecv);
}
std::pair<bool, std::string> Tello::sendCommand(string command){
    std::pair<bool, string> ret;
    bool success = false;
    uint cont = 0;
    const int timeLimit = 10;
    string msgsBack = "";
    cout<<command<<endl;
    do{
        commandSender->sending(command);
        sleep(1);
        msgsBack = commandSender->receiving();
        cont++;
        //cout<<cont<<endl;
    }while((msgsBack.length()==0) and (cont<=timeLimit));

    if (cont>timeLimit){
            cout<<"The command '"<< command <<"' is not received."<<endl;
    }else {
        success = true;
    }
    ret.first = success;
    ret.second = msgsBack;
    return ret; 
}

void Tello::threadStateFnc (){
    pair<bool, vector<double>> resp;
    
    for (;;){
        resp = getState();
        sleep(0.2);
    }
}
std::pair<bool, vector<double>> Tello::getState() {
    const int timeLimit = 10;
    string msgs = "";
    pair<bool, vector<double>> ret;
    ret.first = true;

    msgs = stateRecv->receiving();
    
    if (msgs.length()==0){
        ret.first = false;
    }
    else
        ret.first = filterState(msgs);

    ret.second = state;
    return ret;
}

bool Tello::filterState(string data){
    bool success = true;
    vector<string> values, values_;
    boost::split(values, data, boost::is_any_of(";"));
    for (int value_idx = 0; value_idx<values.size(); value_idx++){
        if (values[value_idx].length()!=0){
            boost::split(values_, values[value_idx], boost::is_any_of(":"));
            //cout<<values[value_idx]<<endl;
            if (value_idx<=state.size()){
                state[value_idx] = stod(values_[1]);
            }else{
                cout<<"Error: Adding data to the 'state' attribute"<<endl;
                success = false;
            }
        }
    }
    if (success)
        update();
    return success;  
}
void Tello::update(){

    orientation.x = state[0];
    orientation.y = state[1];
    orientation.z = state[2];

    velocity.x = state[3];
    velocity.y = state[4];
    velocity.z = state[5];

    timeOF = state[8];
    height = state[9];
    battery = (int)state[10];
    timeMotor = state[12];
    barometer = state [11];

    acceleration.x = state[13];
    acceleration.y = state[14];
    acceleration.z = state[15];

}
coordinates Tello::getOrientation (){
    return orientation;
}

coordinates Tello::getVelocity (){
    return velocity;
}

coordinates Tello::getAcceleration(){
    return acceleration;
}
double Tello::getBarometer(){
    return barometer;
}
vector<coordinates> Tello::getIMU(){
    vector<coordinates> imu;
    imu.push_back(orientation);
    imu.push_back(velocity);
    imu.push_back(acceleration);
    return imu;

}
double Tello::getHeight(){
    return height;
}
double Tello::getBattery(){
    return battery;
}

bool Tello::isConnected(){
    return connected;
}

void Tello::streamVideo(){
    pair<bool, string> response = sendCommand("streamon");

    if (response.first){
        VideoCapture capture{URL_stream, CAP_FFMPEG};
        while (true)
        {
            capture >> frame;
            
            if (!frame.empty())
            {
                imshow("Tello Stream", frame);
            }
            if (waitKey(1) == 27)
            {
                break;
            }
        }
    }
}
// Forward or backward move. 
bool Tello::x_motion(double x){
    std::pair<bool, std::string> response;
    string msg;
    response.first = false; 
    response.second = "";
    bool ret = false;
    if (x>0){
        msg = "forward "+ to_string(abs(x));
        response = sendCommand(msg);
    }else if (x<0){
        msg = "back "+ to_string(abs(x));
        response = sendCommand(msg);
    }
    else{
        response.first = true; 
    }
    if (response.first and response.second != "Error"){
        ret = true;
    }
    return ret;
}

// right or left move. 
bool Tello::y_motion(double y){
    std::pair<bool, std::string> response;
    string msg; 
    response.first = false;
    response.second = "";
    bool ret = false;
    if (y>0){
        msg = "right "+ to_string(abs(y));
        response = sendCommand(msg);
    }else if (y<0){
        msg = "left "+ to_string(abs(y));
        response = sendCommand(msg);
    }
    else{
        response.first = true; 
    }
    if (response.first and response.second != "Error"){
        ret = true;
    }
    return ret;
}

// up or left down. 
bool Tello::z_motion(double z){
    std::pair<bool, std::string> response;
    string msg; 
    response.first = false;
    response.second = "";
    bool ret = false;
    if (z>0){
        msg = "up "+ to_string(int(abs(z)));
        response = sendCommand(msg);
    }else if (z<0){
        msg = "down "+ to_string(int(abs(z)));
        response = sendCommand(msg);
    }
    else{
        response.first = true; 
    }
    if (response.first and response.second != "Error"){
        ret = true;
    }
    return ret;
}

// clockwise or counterclockwise
bool Tello::yaw_twist(double yaw){
    std::pair<bool, std::string> response;
    string msg; 
    response.first = false;
    response.second = "";
    bool ret = false;
    if (yaw>0){
        msg = "cw "+ to_string(abs(yaw));
        response = sendCommand(msg);
    }else if (yaw<0){
        msg = "ccw "+ to_string(abs(yaw));
        response = sendCommand(msg);
    }
    else{
        response.first = true; 
    }
    if (response.first and response.second != "Error"){
        ret = true;
    }
    return ret;
}

bool Tello::speedMotion(double x, double y, double z, double yaw){
    bool ret = false;
    std::pair<bool, std::string> response;
    string msg; 
    response.first = false;
    response.second = "";

    msg = "rc " + to_string(int(x)) + " " + to_string(int(y)) + " " +
            to_string(int(z)) + " " + to_string(int(yaw));
    response = sendCommand(msg);

    if (response.first and response.second != "Error"){
        ret = true;
    }
    return ret;
}