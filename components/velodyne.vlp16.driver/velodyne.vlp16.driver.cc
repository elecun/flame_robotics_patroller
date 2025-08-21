
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

        VLP16Params params;
        params.data_port = parameters.value("data_port", 2368);
        params.bind_ip = parameters.value("bind_ip", "0.0.0.0");
        params.recv_timeout_ms = parameters.value("recv_timeout_ms", 100);
        params.use_device_ip_filter = parameters.value("use_device_ip_filter", false);
        params.device_ip = parameters.value("device_ip", "");
        params.max_packet_per_cycle = parameters.value("max_packet_per_cycle", 400);
        params.use_azimuth_cycle = parameters.value("use_azimuth_cycle", false);
        params.azimuth_field_offset = parameters.value("azimuth_field_offset", 100);
        params.verbose = parameters.value("verbose", false);

        

    }
    catch(json::exception& e){
        logger::error("Profile Error : {}", e.what());
        return false;
    }

    return true;
}

void synerex_rtk_receiver::on_loop(){

    // reader.set_cycle_callback([](const std::vector<VLP16Packet>& cycle){
    //     // 1사이클 수신 완료 시 처리(파싱/포인트 변환/저장 등)
    //     std::cout << "Cycle received: " << cycle.size() << " packets" << std::endl;
    // });

}


void synerex_rtk_receiver::on_close(){

    _bus.close();

}

void synerex_rtk_receiver::on_message(){
    
}



