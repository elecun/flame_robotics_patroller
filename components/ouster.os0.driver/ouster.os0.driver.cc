
#include "ouster.os0.driver.hpp"
#include <flame/log.hpp>
#include <flame/config_def.hpp>
#include <chrono>

using namespace flame;
using namespace std;

/* create component instance */
static ouster_os0_driver* _instance = nullptr;
flame::component::object* create(){ if(!_instance) _instance = new ouster_os0_driver(); return _instance; }
void release(){ if(_instance){ delete _instance; _instance = nullptr; }}


bool ouster_os0_driver::on_init(){

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

void ouster_os0_driver::on_loop(){

    // reader.set_cycle_callback([](const std::vector<VLP16Packet>& cycle){
    //     // 1사이클 수신 완료 시 처리(파싱/포인트 변환/저장 등)
    //     std::cout << "Cycle received: " << cycle.size() << " packets" << std::endl;
    // });

}


void ouster_os0_driver::on_close(){

    _bus.close();

}

void ouster_os0_driver::on_message(){
    
}



