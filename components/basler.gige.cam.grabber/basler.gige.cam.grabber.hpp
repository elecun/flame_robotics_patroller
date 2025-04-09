/**
 * @file basler.gige.cam.grabber.hpp
 * @author Byunghun Hwang <bh.hwang@iae.re.kr>
 * @brief Basler Gigabit Ethernet Camera Grabber
 * @version 0.1
 * @date 2024-06-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef FLAME_BASLER_GIGE_CAM_GRABBER_HPP_INCLUDED
#define FLAME_BASLER_GIGE_CAM_GRABBER_HPP_INCLUDED

#include <flame/component/object.hpp>
#include <map>
#include <unordered_map>
#include <vector>
#include <thread>
#include <string>
#include <atomic>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

using namespace std;
using namespace Pylon;
using namespace GenApi;

class basler_gige_cam_grabber : public flame::component::object {
    public:
        basler_gige_cam_grabber() = default;
        virtual ~basler_gige_cam_grabber() = default;

        /* default interface functions */
        bool on_init() override;
        void on_loop() override;
        void on_close() override;
        void on_message() override;


    private:
        /* for device handle */
        map<int, CBaslerUniversalInstantCamera*> _device_map; // (camera id, instance)

        /* for worker threads handle */
        unordered_map<int, thread> _camera_grab_worker; // (camera id, thread)
        atomic<bool> _worker_stop {false};
        atomic<bool> _use_image_stream_monitoring {false};
        atomic<bool> _use_image_stream {false};

    private:
        /* private task */
        void _image_stream_task(int camera_id, CBaslerUniversalInstantCamera* camera); /* image capture & flush in pipeline */ 

}; /* class */

EXPORT_COMPONENT_API


#endif