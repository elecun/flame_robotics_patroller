
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


bool synerex_rtk_receiver::on_init(){

    try{

        /* get parameters from profile */
        json parameters = get_profile()->parameters();

        

    }
    catch(json::exception& e){
        logger::error("Profile Error : {}", e.what());
        return false;
    }

    return true;
}

void synerex_rtk_receiver::on_loop(){

}


void synerex_rtk_receiver::on_close(){

}

void synerex_rtk_receiver::on_message(){
    
}



