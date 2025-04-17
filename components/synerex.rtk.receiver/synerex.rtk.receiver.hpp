/**
 * @file synerex.rtk.receiver.hpp
 * @author byunghun hwang <bh.hwang@iae.re.kr>
 * @brief Synerex RTK Receiver component
 * @version 0.1
 * @date 2025-04-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #ifndef FLAME_SYNEREX_RTK_RECEIVER_HPP_INCLUDED
 #define FLAME_SYNEREX_RTK_RECEIVER_HPP_INCLUDED
 
 #include <flame/component/object.hpp>
 #include <map>
 #include <unordered_map>
 #include <vector>
 #include <thread>
 #include <string>
 #include <atomic>
  
 using namespace std;
 
 class synerex_rtk_receiver : public flame::component::object {
     public:
         basler_gige_cam_grabber() = default;
         virtual ~basler_gige_cam_grabber() = default;
 
         /* default interface functions */
         bool on_init() override;
         void on_loop() override;
         void on_close() override;
         void on_message() override;
 
 
     private:
         atomic<bool> _worker_stop {false};
 
 }; /* class */
 
 EXPORT_COMPONENT_API
 
 
 #endif