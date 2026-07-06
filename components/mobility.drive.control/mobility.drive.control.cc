#include "mobility.drive.control.hpp"
#include <flame/log.hpp>
#include <json.hpp>
#include <chrono>

using namespace flame;
using namespace std;
using json = nlohmann::json;

/* create component instance */
static mobility_drive_control* _instance = nullptr;
flame::component::Object* Create(){ if(!_instance) _instance = new mobility_drive_control(); return _instance; }
void Release(){ if(_instance){ delete _instance; _instance = nullptr; }}

bool mobility_drive_control::onInit(){
    try{
        json parameters = getProfile()->parameters();
        
        _can_channel = parameters.value("can_channel", 0);
        _max_speed_limit = parameters.value("max_speed_limit_kmh", 5.0f);
        _max_steering_angle = parameters.value("max_steering_angle_deg", 24.7f);

        canInitializeLibrary();
        _can_handle = canOpenChannel(_can_channel, canOPEN_ACCEPT_VIRTUAL);
        if(_can_handle < 0){
            char err[512] = {0,};
            canGetErrorText((canStatus)_can_handle, err, sizeof(err));
            logger::error("[{}] Failed to open CAN Channel : {}", getName(), err);
            return false;
        }

        unsigned long bitrate = parameters.value("can_bitrate", 500000);
        canStatus stat;
        switch(bitrate){
            case 1000000: stat = canSetBusParams(_can_handle, canBITRATE_1M, 0, 0, 0, 0, 0); break;
            case 500000: stat = canSetBusParams(_can_handle, canBITRATE_500K, 0, 0, 0, 0, 0); break;
            case 250000: stat = canSetBusParams(_can_handle, canBITRATE_250K, 0, 0, 0, 0, 0); break;
            case 125000: stat = canSetBusParams(_can_handle, canBITRATE_125K, 0, 0, 0, 0, 0); break;
            default:      stat = canSetBusParams(_can_handle, canBITRATE_500K, 0, 0, 0, 0, 0); break;
        }

        if(stat != canOK){
            char err[512] = {0,};
            canGetErrorText(stat, err, sizeof(err));
            logger::error("[{}] Failed to set bitrate : {}", getName(), err);
            return false;
        }

        stat = canBusOn(_can_handle);
        if(stat != canOK){
            logger::error("[{}] Failed to go bus ON", getName());
            return false;
        }

        _can_rcv_worker = std::thread(&mobility_drive_control::_can_rcv_task, this);
        logger::info("[{}] Initialized successfully.", getName());

    } catch(json::exception& e){
        logger::error("[{}] Profile Error : {}", getName(), e.what());
        return false;
    }

    return true;
}

void mobility_drive_control::onLoop(){
    float speed = _target_speed.load();
    float angle = _target_angle.load();
    int gear = _current_gear.load();

    // constrain speed
    if (speed > _max_speed_limit) speed = _max_speed_limit;
    if (speed < -_max_speed_limit) speed = -_max_speed_limit;

    // constrain angle
    if (angle > _max_steering_angle) angle = _max_steering_angle;
    if (angle < -_max_steering_angle) angle = -_max_steering_angle;

    auto msgs = _driver.set_drive_command(speed, angle, gear);
    for(const auto& m : msgs){
        if (_can_handle >= 0) {
            canWrite(_can_handle, m.id, (void*)m.data, m.dlc, 0);
        }
    }
}

void mobility_drive_control::onClose(){
    _worker_stop.store(true);
    if(_can_rcv_worker.joinable()){
        _can_rcv_worker.join();
    }
    if(_can_handle >= 0){
        canBusOff(_can_handle);
        canClose(_can_handle);
        _can_handle = -1;
    }
    canUnloadLibrary();
}

void mobility_drive_control::onData(flame::component::ZData& data){
    try {
        json j;
        while (!data.empty()) {
            string s = data.popstr();
            try { j = json::parse(s); } catch(...) {}
        }
        
        if (j.contains("command")) {
            string cmd = j["command"];
            if (cmd == "drive") {
                if (j.contains("angle")) _target_angle.store(j["angle"]);
                if (j.contains("velocity")) _target_speed.store(j["velocity"]);
                _current_gear.store(-1);
            } else if (cmd == "stop") {
                _target_angle.store(0.0f);
                _target_speed.store(0.0f);
                _current_gear.store(-1);
            } else if (cmd == "forward") {
                _current_gear.store(1); // D
            } else if (cmd == "backward") {
                _current_gear.store(3); // R
            } else if (cmd == "neutral") {
                _current_gear.store(2); // N
                _target_speed.store(0.0f);
            }
        }
    } catch (...) {
        logger::error("[{}] Error parsing onData", getName());
    }
}

void mobility_drive_control::_can_rcv_task(){
    while(!_worker_stop.load()){
        long id;
        unsigned char data[8];
        unsigned int dlc;
        unsigned int flags;
        unsigned long time;

        if (_can_handle >= 0) {
            canStatus stat = canRead(_can_handle, &id, data, &dlc, &flags, &time);
            if(stat == canOK) {
                _driver.parse(id, data, dlc);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
