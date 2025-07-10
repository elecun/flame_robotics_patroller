/**
 * @file baumer.inclination.sensor.hpp
 * @author Byunghun Hwang
 * @brief Baumer Inclination Sensor using CANopen with Kvaser CAN Controller(canlib)
 * @version 0.1
 * @date 2025-07-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef FLAME_BAUMER_INCLINATION_SENSOR_HPP_INCLUDED
#define FLAME_BAUMER_INCLINATION_SENSOR_HPP_INCLUDED

#include <flame/component/object.hpp>
#include <map>
#include <unordered_map>
#include <vector>
#include <thread>
#include <string>
#include <atomic>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
extern "C" {
    #include <canlib.h>
}


using namespace std;


class baumer_inclination_sensor : public flame::component::object {
    public:
        baumer_inclination_sensor() = default;
        virtual ~baumer_inclination_sensor() = default;

        /* default interface functions */
        bool on_init() override;
        void on_loop() override;
        void on_close() override;
        void on_message() override;

    private:
        /* CAN Receive Task function */
        void _can_rcv_task();

        /* activate PDO */
        void _send_nmt_start_remote_node();

        /* string format from double  */
        string _set_precision_str(double value, int precision);

    private:
        int _can_channel {0};
        CanHandle _can_handle { canINVALID_HANDLE };
        thread _can_rcv_worker;
        int _node_id {0};

        atomic<bool> _worker_stop { false };

}; /* class */

EXPORT_COMPONENT_API


#endif