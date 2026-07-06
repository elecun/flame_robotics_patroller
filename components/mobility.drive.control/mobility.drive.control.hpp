#ifndef FLAME_MOBILITY_DRIVE_CONTROL_HPP_INCLUDED
#define FLAME_MOBILITY_DRIVE_CONTROL_HPP_INCLUDED

#include <flame/component/object.hpp>
#include <atomic>
#include <thread>
#include <vector>
#include "s1_driver.hpp"
#include <canlib.h>

class mobility_drive_control : public flame::component::Object {
public:
    mobility_drive_control() = default;
    virtual ~mobility_drive_control() = default;

    bool onInit() override;
    void onLoop() override;
    void onClose() override;
    void onData(flame::component::ZData& data) override;

private:
    void _can_rcv_task();

private:
    s1_driver::S1Driver _driver;
    int _can_channel = 0;
    canHandle _can_handle = -1;
    std::thread _can_rcv_worker;
    std::atomic<bool> _worker_stop{false};

    std::atomic<float> _target_speed{0.0f};
    std::atomic<float> _target_angle{0.0f};
    std::atomic<int> _current_gear{-1}; // -1: Auto, 1: D, 2: N, 3: R

    float _max_speed_limit = 5.0f;
    float _max_steering_angle = 24.7f;
};

EXPORT_COMPONENT_API

#endif