/**
 * @file velodyne.vlp16.driver.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef FLAME_VELODYNE_VLP16_DRIVER_HPP_INCLUDED
#define FLAME_VELODYNE_VLP16_DRIVER_HPP_INCLUDED

#include <flame/component/object.hpp>
#include <signal.h>
#include "vlp16.hpp"

using namespace std;

class velodyne_vlp16_driver : public flame::component::object {
    public:
    velodyne_vlp16_driver() = default;
        virtual ~velodyne_vlp16_driver() = default;

        /* default interface functions */
        bool on_init() override;
        void on_loop() override;
        void on_close() override;
        void on_message() override;

    private:
        VLP16Reader* g_reader = nullptr;

        atomic<bool> _worker_stop {false};


}; /* class */

EXPORT_COMPONENT_API


#endif