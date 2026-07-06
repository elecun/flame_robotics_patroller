#ifndef FLAME_MOBILITY_DRIVE_CONTROL_S1_DRIVER_HPP_INCLUDED
#define FLAME_MOBILITY_DRIVE_CONTROL_S1_DRIVER_HPP_INCLUDED

#include <cstdint>
#include <vector>
#include <string>
#include <cmath>

namespace s1_driver {

struct can_message {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    bool is_extended;
};

class S1Driver {
public:
    S1Driver() = default;
    ~S1Driver() = default;

    // CAN 메시지 파싱
    void parse(uint32_t can_id, const uint8_t* data, size_t dlc) {
        if (dlc == 0) return;

        switch (can_id) {
            case 0x303:
                if (dlc >= 4) {
                    vehicle_gear_ = data[0] & 0x03;
                    drive_state_mode_ = data[1] & 0x03;
                    vehicle_speed_request_ = (data[2] | (data[3] << 8)) * 0.1f - 80.0f;
                }
                break;
            case 0x314:
                if (dlc >= 3) {
                    direction_angle_ = (data[1] | (data[2] << 8));
                    eps_control_ = (data[0] & 0x01) ? true : false;
                }
                break;
            case 0x304:
                if (dlc >= 6) {
                    vehicle_speed_ = (data[0] | (data[1] << 8)) * 0.1f - 80.0f;
                    break_pressure_ = (data[2] | (data[3] << 8)) * 0.01f;
                    wheel_end_angle_ = (data[4] | (data[5] << 8)) * 0.1f - 35.0f;
                }
                break;
            case 0x301:
                if (dlc >= 6) {
                    emergency_button_ = (data[0] & 0x01) ? true : false;
                    head_light_ = (data[1] & 0x80) ? true : false;
                    back_touch_switch_ = (data[1] & 0x20) ? true : false;
                    front_touch_switch_ = (data[1] & 0x10) ? true : false;
                    brake_light_ = (data[5] & 0x01) ? true : false;
                }
                break;
            case 0x18F:
                if (dlc >= 7) {
                    int16_t angle_raw = data[1] | (data[2] << 8);
                    eps_current_angle_ = angle_raw;
                    eps_ecu_temperature_ = static_cast<int8_t>(data[6]);
                }
                break;
            case 0x060:
                if (dlc >= 4) {
                    bus_voltage_ = (data[0] | (data[1] << 8)) * 0.1f;
                    bus_current_ = (data[2] | (data[3] << 8)) * 0.1f - 1000.0f;
                }
                break;
            case 0x160:
                if (dlc >= 6) {
                    drive_mode_ = (data[0] & 0x06) >> 1;
                    mcu_brake_request_ = (data[0] & 0x08) ? true : false;
                    mcu_torque_request_ = (data[1] | (data[2] << 8)) * 0.1f - 1000.0f;
                    mcu_speed_request_ = (data[3] | (data[4] << 8) | (data[5] << 16)) - 7000;
                }
                break;
            case 0x0A0:
                if (dlc >= 8) {
                    bms_battery_voltage_ = (data[2] | (data[3] << 8)) * 0.1f;
                    bms_battery_soc_ = data[4] * 0.4f;
                    bms_battery_soh_ = data[7];
                }
                break;
            default:
                break;
        }
    }

    // 제어 명령 수행 (set_ prefix)
    std::vector<can_message> set_drive_command(float speed, float angular, int override_gear = -1) {
        std::vector<can_message> msgs;
        
        // 각도 제한 (-30 ~ 30)
        if (angular < -30.0f) angular = -30.0f;
        if (angular > 30.0f) angular = 30.0f;

        // 기어 설정 (P:0, D:1, N:2, R:3)
        uint8_t gear = 0x02; // 기본 N
        if (override_gear != -1) {
            gear = override_gear;
            speed = std::abs(speed);
        } else {
            if (speed > 0.1f) {
                gear = 0x01; // D
            } else if (speed < -0.1f) {
                gear = 0x03; // R
                speed = std::abs(speed);
            }
        }

        // 방향 지시등 설정 (None:0, Left:0xF1, Right:0xF2)
        uint8_t indicator = 0x00;
        if (angular < -5.0f) {
            indicator = 0xF2; // 우회전
        } else if (angular > 5.0f) {
            indicator = 0xF1; // 좌회전
        }

        // VCU_Speed_Req
        int speed_val_for_504 = static_cast<int>(speed / 0.1f);
        uint8_t linear_v1 = speed_val_for_504 & 0xFF;
        uint8_t linear_v2 = (speed_val_for_504 >> 8) & 0xFF;

        // 조향 각도 제어 (angular_val = (angular + 30) / 0.1)
        int angular_val_for_502 = static_cast<int>((angular + 30.0f) / 0.1f);
        uint8_t angular_v1 = angular_val_for_502 & 0xFF;
        uint8_t angular_v2 = (angular_val_for_502 >> 8) & 0xFF;

        msgs.push_back({0x501, {0xF1, 0, 0, 0, 0, 0, 0, 0}, 8, false});
        msgs.push_back({0x503, {0xF1, 0, 0, 0, 0, 0, 0, 0}, 8, false});
        msgs.push_back({0x502, {0xF1, 0, 0, 0, angular_v1, angular_v2, 0, 0}, 8, false});
        msgs.push_back({0x506, {indicator, 0, 0, 0, 0, 0, 0, 0}, 8, false});
        msgs.push_back({0x504, {0xF1, 0x00, 0x01, gear, 0, 0, linear_v1, linear_v2}, 8, false});

        return msgs;
    }

    // 상태 읽기 (get_ prefix)
    int get_vehicle_gear() const { return vehicle_gear_; }
    int get_drive_state_mode() const { return drive_state_mode_; }
    float get_vehicle_speed_request() const { return vehicle_speed_request_; }
    
    int get_direction_angle() const { return direction_angle_; }
    bool get_eps_control() const { return eps_control_; }
    
    float get_vehicle_speed() const { return vehicle_speed_; }
    float get_wheel_end_angle() const { return wheel_end_angle_; }
    float get_break_pressure() const { return break_pressure_; }
    
    bool get_brake_light() const { return brake_light_; }
    bool get_head_light() const { return head_light_; }
    bool get_emergency_button() const { return emergency_button_; }
    bool get_back_touch_switch() const { return back_touch_switch_; }
    bool get_front_touch_switch() const { return front_touch_switch_; }
    
    int get_eps_current_angle() const { return eps_current_angle_; }
    int get_eps_ecu_temperature() const { return eps_ecu_temperature_; }
    
    float get_bus_voltage() const { return bus_voltage_; }
    float get_bus_current() const { return bus_current_; }
    
    int get_drive_mode() const { return drive_mode_; }
    bool get_mcu_brake_request() const { return mcu_brake_request_; }
    int get_mcu_speed_request() const { return mcu_speed_request_; }
    float get_mcu_torque_request() const { return mcu_torque_request_; }
    
    int get_bms_battery_soh() const { return bms_battery_soh_; }
    float get_bms_battery_soc() const { return bms_battery_soc_; }
    float get_bms_battery_voltage() const { return bms_battery_voltage_; }

private:
    int vehicle_gear_ = 0;
    int drive_state_mode_ = 0;
    float vehicle_speed_request_ = 0.0f;
    
    int direction_angle_ = 0;
    bool eps_control_ = false;
    
    float vehicle_speed_ = 0.0f;
    float wheel_end_angle_ = 0.0f;
    float break_pressure_ = 0.0f;
    
    bool brake_light_ = false;
    bool head_light_ = false;
    bool emergency_button_ = false;
    bool back_touch_switch_ = false;
    bool front_touch_switch_ = false;
    
    int eps_current_angle_ = 0;
    int eps_ecu_temperature_ = 0;
    
    float bus_voltage_ = 0.0f;
    float bus_current_ = 0.0f;
    
    int drive_mode_ = 0;
    bool mcu_brake_request_ = false;
    int mcu_speed_request_ = 0;
    float mcu_torque_request_ = 0.0f;
    
    int bms_battery_soh_ = 0;
    float bms_battery_soc_ = 0.0f;
    float bms_battery_voltage_ = 0.0f;
};

} // namespace s1_driver

#endif // FLAME_MOBILITY_DRIVE_CONTROL_S1_DRIVER_HPP_INCLUDED
