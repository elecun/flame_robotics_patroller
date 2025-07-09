
#include "baumer.inclination.sensor.hpp"
#include <flame/log.hpp>
#include <flame/config_def.hpp>
#include <chrono>

using namespace flame;
using namespace std;

/* create component instance */
static baumer_inclination_sensor* _instance = nullptr;
flame::component::object* create(){ if(!_instance) _instance = new baumer_inclination_sensor(); return _instance; }
void release(){ if(_instance){ delete _instance; _instance = nullptr; }}


bool baumer_inclination_sensor::on_init(){

    try{

        /* get parameters from profile */
        json parameters = get_profile()->parameters();

        _node_id = parameters.value("node_id", 0);

        /* initialize */
        canInitializeLibrary();

        /* open CAN channel */
        _can_channel = parameters.value("can_channel", 0);
        _can_handle = canOpenChannel(_can_channel, canOPEN_ACCEPT_VIRTUAL);
        if(_can_handle<0){
            char err[512] = {0,};
            canGetErrorText((canStatus)_can_handle, err, sizeof(err));
            logger::error("[{}] Failed to open CAN Channel : {}", get_name(), err);
            return false;
        }

        /* set bitrate */
        canStatus stat = canSetBusParams(_can_handle, canBITRATE_500K, 0, 0, 0, 0, 0);
        if(stat!=canOK){
            char err[512] = {0,};
            canGetErrorText(stat, err, sizeof(err));
            logger::error("[{}] Failed to set bitrate : {}", get_name(), err);
            return false;
        }

        /* bus on */
        stat = canBusOn(_can_handle);
        if(stat!=canOK){
            char err[512] = {0,};
            canGetErrorText(stat, err, sizeof(err));
            logger::error("[{}] Failed to go bus ON : {}", get_name(), err);
            return false;
        }

        /* nmt start remote node */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        _send_nmt_start_remote_node();

        /* start listen */
        logger::info("[{}] Listen for CAN frames...", get_name());
        _can_rcv_worker = thread(&baumer_inclination_sensor::_can_rcv_task, this);

    }
    catch(json::exception& e){
        logger::error("Profile Error : {}", e.what());
        return false;
    }

    return true;
}

void baumer_inclination_sensor::on_loop(){

}


void baumer_inclination_sensor::on_close(){

    _worker_stop.store(true);
    if(_can_rcv_worker.joinable()){
        _can_rcv_worker.join();
        logger::info("[{}] Component successfully closed.", get_name());
    }

}

void baumer_inclination_sensor::on_message(){
    
}

string baumer_inclination_sensor::_set_precision_str(double value, int precision){
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

void baumer_inclination_sensor::_send_nmt_start_remote_node(){
    if(_can_handle>=0){
        long id = 0x000; /* CANOpen NMT Master to Slave */
        unsigned char data[2];
        data[0] = 0x01; /* Start remote node command */
        data[1] = _node_id; /* Node ID */

        canStatus stat = canWrite(_can_handle, id, data, 2, 0);
        if(stat!=canOK) {
            char err[512] = {0,};
            canGetErrorText(stat, err, sizeof(err));
            logger::error("[{}] {}", get_name(), err);
        }
        else {
            logger::info("[{}] Request to activate PDO", get_name());
        }
    }
    else{
        logger::warn("[{}] Invalid CAN Device handle", get_name());
    }
}

void baumer_inclination_sensor::_can_rcv_task(){
    
    try{
        json tag;
        auto last_time = chrono::high_resolution_clock::now();
        const double resolution = 0.1;
        while(!_worker_stop.load()){
            long id;
            unsigned char data[8];
            unsigned int dlc;
            unsigned int flags;
            unsigned long time;

            canStatus stat = canRead(_can_handle, &id, data, &dlc, &flags, &time);
            if(stat==canOK) {
                std::ostringstream oss;
                for(unsigned char d:data){
                    oss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(d) << " ";
                }
                logger::debug("[{}] ID({}) | DLC({}) | Data({})", get_name(), id, dlc, oss.str());

                if(id==(0x180+_node_id)){
                    int16_t temperature = data[0] | (data[1] << 8);
                    int16_t slope_z = data[2] | (data[3] << 8);
                    int16_t slope_y = data[4] | (data[5] << 8);

                    double slope_z_deg = static_cast<double>(slope_z)*resolution;
                    double slope_y_deg = static_cast<double>(slope_y)*resolution;

                    logger::info("[{}] Y({:.3f}), Z({:.3f}), Temp({})", get_name(), slope_y_deg, slope_z_deg, to_string(temperature));
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));


        } /* end while */

        /* realse */
        canBusOff(_can_handle);
        canClose(_can_handle);
        logger::info("[{}] Close Device", get_name());
    }
    catch(const std::out_of_range& e){
        logger::error("[{}] Invalid parameter access", get_name());
    }
    catch(const zmq::error_t& e){
        logger::error("[{}] Piepeline Error : {}", get_name(), e.what());
    }
    catch(const json::exception& e){
        logger::error("[{}] Data Parse Error : {}", get_name(), e.what());
    }
}



