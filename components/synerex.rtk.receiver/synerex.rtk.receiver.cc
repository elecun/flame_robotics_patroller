
#include "synerex.rtk.receiver.hpp"
#include <flame/log.hpp>
#include <flame/config_def.hpp>
#include <chrono>

using namespace flame;
using namespace std;

/* create component instance */
static synerex_rtk_receiver* _instance = nullptr;
flame::component::object* create(){ if(!_instance) _instance = new synerex_rtk_receiver(); return _instance; }
void release(){ if(_instance){ delete _instance; _instance = nullptr; }}


bool synerex_rtk_receiver::onInit(){

    try{

        /* get parameters from profile */
        json parameters = get_profile()->parameters();

        string port = parameters.value("port", "/dev/ttyS0");
        unsigned int baudrate = parameters.value("baudrate", 115200);
        // char err = _serial.openDevice(port.c_str(), baudrate);
        // if(err!=1)
        //     return false;

        

    }
    catch(json::exception& e){
        logger::error("Profile Error : {}", e.what());
        return false;
    }

    return true;
}

void synerex_rtk_receiver::onLoop(){

}


void synerex_rtk_receiver::onClose(){

    _bus.close();

}

void synerex_rtk_receiver::onData(flame::component::ZData& data){
    
}



