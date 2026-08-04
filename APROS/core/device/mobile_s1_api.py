"""
Mobile S1 CAN DBC Protocol API Frame Builders.
"""

class MobileS1API:
    def _s1_api_vcu_vehicle_status_1(
        self,
        vehicle_status_msgcntr: float = 0.0,
        drive_mode_state: float = 0.0,
        vehicle_soc: float = 0.0,
        vcu_speed_req: float = 0.0,
        clamping_brake_status: float = 0.0,
        vehicle_gear: float = 0.0
    ) -> bytearray:
        """
        Build VCU_Vehicle_Status_1 (CAN ID: 0x303) payload bytearray.
        """
        payload = bytearray(8)
        raw_vehicle_status_msgcntr = int(vehicle_status_msgcntr)
        payload[7] |= ((raw_vehicle_status_msgcntr & 0xF) << 4) & 0xFF
        raw_drive_mode_state = int(drive_mode_state)
        payload[1] |= ((raw_drive_mode_state & 0xF) << 0) & 0xFF
        raw_vehicle_soc = int(vehicle_soc)
        payload[6] = raw_vehicle_soc & 0xFF
        raw_vcu_speed_req = int(round((vcu_speed_req - (-80.0)) / 0.1))
        payload[2] = raw_vcu_speed_req & 0xFF
        payload[3] = (raw_vcu_speed_req >> 8) & 0xFF
        raw_clamping_brake_status = int(clamping_brake_status)
        payload[0] |= ((raw_clamping_brake_status & 0x1) << 3) & 0xFF
        raw_vehicle_gear = int(vehicle_gear)
        payload[0] |= ((raw_vehicle_gear & 0x3) << 0) & 0xFF
        return payload

    def _s1_api_parallel_control_flag(
        self,
        parallel_flag_msgcntr: float = 0.0,
        parallel_control_request_flag: float = 0.0
    ) -> bytearray:
        """
        Build Parallel_Control_Flag (CAN ID: 0x511) payload bytearray.
        """
        payload = bytearray(8)
        raw_parallel_flag_msgcntr = int(parallel_flag_msgcntr)
        payload[0] |= ((raw_parallel_flag_msgcntr & 0xF) << 4) & 0xFF
        raw_parallel_control_request_flag = int(parallel_control_request_flag)
        payload[0] |= ((raw_parallel_control_request_flag & 0xF) << 0) & 0xFF
        return payload

    def _s1_api_parallel_control_body(
        self,
        parallel_brakelight_switch: float = 0.0,
        parallel_body_msgcntr: float = 0.0,
        parallel_hom_switch: float = 0.0,
        parallel_headlight_switch: float = 0.0,
        parallel_right_light_switch: float = 0.0,
        parallel_left_light_switch: float = 0.0
    ) -> bytearray:
        """
        Build Parallel_Control_Body (CAN ID: 0x516) payload bytearray.
        """
        payload = bytearray(8)
        raw_parallel_brakelight_switch = int(parallel_brakelight_switch)
        payload[1] |= ((raw_parallel_brakelight_switch & 0x1) << 0) & 0xFF
        raw_parallel_body_msgcntr = int(parallel_body_msgcntr)
        payload[0] |= ((raw_parallel_body_msgcntr & 0xF) << 4) & 0xFF
        raw_parallel_hom_switch = int(parallel_hom_switch)
        payload[0] |= ((raw_parallel_hom_switch & 0x1) << 2) & 0xFF
        raw_parallel_headlight_switch = int(parallel_headlight_switch)
        payload[0] |= ((raw_parallel_headlight_switch & 0x1) << 3) & 0xFF
        raw_parallel_right_light_switch = int(parallel_right_light_switch)
        payload[0] |= ((raw_parallel_right_light_switch & 0x1) << 1) & 0xFF
        raw_parallel_left_light_switch = int(parallel_left_light_switch)
        payload[0] |= ((raw_parallel_left_light_switch & 0x1) << 0) & 0xFF
        return payload

    def _s1_api_parallel_control_steering(
        self,
        parallel_steering_msgcntr: float = 0.0,
        parallel_steering_valid: float = 0.0,
        parallel_steering_angle_cmd: float = 0.0
    ) -> bytearray:
        """
        Build Parallel_Control_Steering (CAN ID: 0x512) payload bytearray.
        """
        payload = bytearray(8)
        raw_parallel_steering_msgcntr = int(parallel_steering_msgcntr)
        payload[0] |= ((raw_parallel_steering_msgcntr & 0xF) << 4) & 0xFF
        raw_parallel_steering_valid = int(parallel_steering_valid)
        payload[0] |= ((raw_parallel_steering_valid & 0xF) << 0) & 0xFF
        raw_parallel_steering_angle_cmd = int(round((parallel_steering_angle_cmd - (-30.0)) / 0.1))
        payload[4] = raw_parallel_steering_angle_cmd & 0xFF
        payload[5] = (raw_parallel_steering_angle_cmd >> 8) & 0xFF
        return payload

    def _s1_api_parallel_control_accelerate(
        self,
        parallel_accelerate_msgcntr: float = 0.0,
        parallel_acc_de: float = 0.0,
        parallel_driving_speed_control: float = 0.0,
        parallel_driving_torque_control: float = 0.0,
        parallel_accelerate_gear: float = 0.0,
        parallel_accelerate_work_mode: float = 0.0,
        parallel_accelerate_valid: float = 0.0
    ) -> bytearray:
        """
        Build Parallel_Control_Accelerate (CAN ID: 0x514) payload bytearray.
        """
        payload = bytearray(8)
        raw_parallel_accelerate_msgcntr = int(parallel_accelerate_msgcntr)
        payload[0] |= ((raw_parallel_accelerate_msgcntr & 0xF) << 4) & 0xFF
        raw_parallel_acc_de = int(round((parallel_acc_de - (-5.0)) / 0.1))
        payload[4] = raw_parallel_acc_de & 0xFF
        raw_parallel_driving_speed_control = int(round((parallel_driving_speed_control - (0.0)) / 0.1))
        payload[6] = raw_parallel_driving_speed_control & 0xFF
        payload[7] = (raw_parallel_driving_speed_control >> 8) & 0xFF
        raw_parallel_driving_torque_control = int(parallel_driving_torque_control)
        payload[5] = raw_parallel_driving_torque_control & 0xFF
        raw_parallel_accelerate_gear = int(parallel_accelerate_gear)
        payload[3] = raw_parallel_accelerate_gear & 0xFF
        raw_parallel_accelerate_work_mode = int(parallel_accelerate_work_mode)
        payload[2] = raw_parallel_accelerate_work_mode & 0xFF
        raw_parallel_accelerate_valid = int(parallel_accelerate_valid)
        payload[0] |= ((raw_parallel_accelerate_valid & 0xF) << 0) & 0xFF
        return payload

    def _s1_api_parallel_control_brake(
        self,
        parallel_dbs_msgcntr: float = 0.0,
        parallel_brakepressure_cmd: float = 0.0,
        parallel_dbs_valid: float = 0.0
    ) -> bytearray:
        """
        Build Parallel_Control_Brake (CAN ID: 0x513) payload bytearray.
        """
        payload = bytearray(8)
        raw_parallel_dbs_msgcntr = int(parallel_dbs_msgcntr)
        payload[0] |= ((raw_parallel_dbs_msgcntr & 0xF) << 4) & 0xFF
        raw_parallel_brakepressure_cmd = int(parallel_brakepressure_cmd)
        payload[1] = raw_parallel_brakepressure_cmd & 0xFF
        raw_parallel_dbs_valid = int(parallel_dbs_valid)
        payload[0] |= ((raw_parallel_dbs_valid & 0xF) << 0) & 0xFF
        return payload

    def _s1_api_vcu_eps_control_request(
        self,
        vcu_eps_strangle_req: float = 0.0,
        vcu_eps_ctrlenable: float = 0.0,
        vcu_eps_calibration_req: float = 0.0
    ) -> bytearray:
        """
        Build VCU_EPS_Control_Request (CAN ID: 0x314) payload bytearray.
        """
        payload = bytearray(8)
        raw_vcu_eps_strangle_req = int(vcu_eps_strangle_req)
        payload[1] |= ((raw_vcu_eps_strangle_req & 0xFFFF) << 7) & 0xFF
        raw_vcu_eps_ctrlenable = int(vcu_eps_ctrlenable)
        payload[0] |= ((raw_vcu_eps_ctrlenable & 0x1) << 0) & 0xFF
        raw_vcu_eps_calibration_req = int(vcu_eps_calibration_req)
        payload[0] |= ((raw_vcu_eps_calibration_req & 0x1) << 2) & 0xFF
        return payload

    def _s1_api_ad_control_body(
        self,
        ad_brake_light: float = 0.0,
        ad_body_msgcntr: float = 0.0,
        ad_horn_control: float = 0.0,
        ad_headlight: float = 0.0,
        ad_right_turn_light: float = 0.0,
        ad_left_turn_light: float = 0.0
    ) -> bytearray:
        """
        Build AD_Control_Body (CAN ID: 0x506) payload bytearray.
        """
        payload = bytearray(8)
        raw_ad_brake_light = int(ad_brake_light)
        payload[1] |= ((raw_ad_brake_light & 0x1) << 0) & 0xFF
        raw_ad_body_msgcntr = int(ad_body_msgcntr)
        payload[0] |= ((raw_ad_body_msgcntr & 0xF) << 4) & 0xFF
        raw_ad_horn_control = int(ad_horn_control)
        payload[0] |= ((raw_ad_horn_control & 0x1) << 2) & 0xFF
        raw_ad_headlight = int(ad_headlight)
        payload[0] |= ((raw_ad_headlight & 0x1) << 3) & 0xFF
        raw_ad_right_turn_light = int(ad_right_turn_light)
        payload[0] |= ((raw_ad_right_turn_light & 0x1) << 1) & 0xFF
        raw_ad_left_turn_light = int(ad_left_turn_light)
        payload[0] |= ((raw_ad_left_turn_light & 0x1) << 0) & 0xFF
        return payload

    def _s1_api_vcu_vehicle_status_2(
        self,
        vehicle_status_2_msgcntr: float = 0.0,
        vehicle_steering_angle: float = 0.0,
        vehicle_brake_pressure: float = 0.0,
        vehicle_speed: float = 0.0
    ) -> bytearray:
        """
        Build VCU_Vehicle_Status_2 (CAN ID: 0x304) payload bytearray.
        """
        payload = bytearray(8)
        raw_vehicle_status_2_msgcntr = int(vehicle_status_2_msgcntr)
        payload[7] |= ((raw_vehicle_status_2_msgcntr & 0xF) << 4) & 0xFF
        raw_vehicle_steering_angle = int(round((vehicle_steering_angle - (-35.0)) / 0.1))
        payload[4] |= ((raw_vehicle_steering_angle & 0x3FF) << 0) & 0xFF
        raw_vehicle_brake_pressure = int(round((vehicle_brake_pressure - (0.0)) / 0.01))
        payload[2] = raw_vehicle_brake_pressure & 0xFF
        payload[3] = (raw_vehicle_brake_pressure >> 8) & 0xFF
        raw_vehicle_speed = int(round((vehicle_speed - (-80.0)) / 0.1))
        payload[0] = raw_vehicle_speed & 0xFF
        payload[1] = (raw_vehicle_speed >> 8) & 0xFF
        return payload

    def _s1_api_ad_control_accelerate(
        self,
        ad_accelerate_msgcntr: float = 0.0,
        ad_acc_de: float = 0.0,
        ad_speed_control: float = 0.0,
        ad_torque_control: float = 0.0,
        ad_accelerate_gear: float = 0.0,
        ad_accelerate_work_mode: float = 0.0,
        ad_accelerate_valid: float = 0.0
    ) -> bytearray:
        """
        Build AD_Control_Accelerate (CAN ID: 0x504) payload bytearray.
        """
        payload = bytearray(8)
        raw_ad_accelerate_msgcntr = int(ad_accelerate_msgcntr)
        payload[0] |= ((raw_ad_accelerate_msgcntr & 0xF) << 4) & 0xFF
        raw_ad_acc_de = int(round((ad_acc_de - (-5.0)) / 0.1))
        payload[4] = raw_ad_acc_de & 0xFF
        raw_ad_speed_control = int(round((ad_speed_control - (0.0)) / 0.1))
        payload[6] = raw_ad_speed_control & 0xFF
        payload[7] = (raw_ad_speed_control >> 8) & 0xFF
        raw_ad_torque_control = int(ad_torque_control)
        payload[5] = raw_ad_torque_control & 0xFF
        raw_ad_accelerate_gear = int(ad_accelerate_gear)
        payload[3] = raw_ad_accelerate_gear & 0xFF
        raw_ad_accelerate_work_mode = int(ad_accelerate_work_mode)
        payload[2] = raw_ad_accelerate_work_mode & 0xFF
        raw_ad_accelerate_valid = int(ad_accelerate_valid)
        payload[0] |= ((raw_ad_accelerate_valid & 0xF) << 0) & 0xFF
        return payload

    def _s1_api_ad_control_brake(
        self,
        ad_dbs_msgcntr: float = 0.0,
        ad_brakepressure_cmd: float = 0.0,
        ad_dbs_valid: float = 0.0
    ) -> bytearray:
        """
        Build AD_Control_Brake (CAN ID: 0x503) payload bytearray.
        """
        payload = bytearray(8)
        raw_ad_dbs_msgcntr = int(ad_dbs_msgcntr)
        payload[0] |= ((raw_ad_dbs_msgcntr & 0xF) << 4) & 0xFF
        raw_ad_brakepressure_cmd = int(ad_brakepressure_cmd)
        payload[1] = raw_ad_brakepressure_cmd & 0xFF
        raw_ad_dbs_valid = int(ad_dbs_valid)
        payload[0] |= ((raw_ad_dbs_valid & 0xF) << 0) & 0xFF
        return payload

    def _s1_api_ad_control_steering(
        self,
        ad_steering_msgcntr: float = 0.0,
        ad_steering_angle_cmd: float = 0.0,
        ad_steering_valid: float = 0.0
    ) -> bytearray:
        """
        Build AD_Control_Steering (CAN ID: 0x502) payload bytearray.
        """
        payload = bytearray(8)
        raw_ad_steering_msgcntr = int(ad_steering_msgcntr)
        payload[0] |= ((raw_ad_steering_msgcntr & 0xF) << 4) & 0xFF
        raw_ad_steering_angle_cmd = int(round((ad_steering_angle_cmd - (-30.0)) / 0.1))
        payload[4] = raw_ad_steering_angle_cmd & 0xFF
        payload[5] = (raw_ad_steering_angle_cmd >> 8) & 0xFF
        raw_ad_steering_valid = int(ad_steering_valid)
        payload[0] |= ((raw_ad_steering_valid & 0xF) << 0) & 0xFF
        return payload

    def _s1_api_ad_control_flag(
        self,
        ad_flag_msgcntr: float = 0.0,
        ad_control_request_flag: float = 0.0
    ) -> bytearray:
        """
        Build AD_Control_Flag (CAN ID: 0x501) payload bytearray.
        """
        payload = bytearray(8)
        raw_ad_flag_msgcntr = int(ad_flag_msgcntr)
        payload[0] |= ((raw_ad_flag_msgcntr & 0xF) << 4) & 0xFF
        raw_ad_control_request_flag = int(ad_control_request_flag)
        payload[0] |= ((raw_ad_control_request_flag & 0xF) << 0) & 0xFF
        return payload

    def _s1_api_vcu_vehicle_diagnosis(
        self,
        vehicle_voltage: float = 0.0,
        light_states_brake: float = 0.0,
        eps_state: float = 0.0,
        oil_pot_state: float = 0.0,
        vehicle_diagnosis_msgcntr: float = 0.0,
        horn_state: float = 0.0,
        headlight_state: float = 0.0,
        right_turn_light_state: float = 0.0,
        left_turn_light_state: float = 0.0,
        r_touch_switch_state: float = 0.0,
        f_touch_switch_state: float = 0.0,
        bms_state: float = 0.0,
        emergency_button_state: float = 0.0,
        parallel_state: float = 0.0,
        dbs_state: float = 0.0,
        ad_state: float = 0.0,
        remote_state: float = 0.0,
        motor_state: float = 0.0
    ) -> bytearray:
        """
        Build VCU_Vehicle_Diagnosis (CAN ID: 0x301) payload bytearray.
        """
        payload = bytearray(8)
        raw_vehicle_voltage = int(round((vehicle_voltage - (0.0)) / 0.1))
        payload[2] |= ((raw_vehicle_voltage & 0x3FF) << 6) & 0xFF
        raw_light_states_brake = int(light_states_brake)
        payload[5] |= ((raw_light_states_brake & 0x1) << 0) & 0xFF
        raw_eps_state = int(eps_state)
        payload[1] |= ((raw_eps_state & 0x1) << 2) & 0xFF
        raw_oil_pot_state = int(oil_pot_state)
        payload[5] |= ((raw_oil_pot_state & 0x1) << 1) & 0xFF
        raw_vehicle_diagnosis_msgcntr = int(vehicle_diagnosis_msgcntr)
        payload[7] |= ((raw_vehicle_diagnosis_msgcntr & 0xF) << 4) & 0xFF
        raw_horn_state = int(horn_state)
        payload[5] |= ((raw_horn_state & 0x1) << 3) & 0xFF
        raw_headlight_state = int(headlight_state)
        payload[1] |= ((raw_headlight_state & 0x1) << 7) & 0xFF
        raw_right_turn_light_state = int(right_turn_light_state)
        payload[6] |= ((raw_right_turn_light_state & 0x1) << 0) & 0xFF
        raw_left_turn_light_state = int(left_turn_light_state)
        payload[4] |= ((raw_left_turn_light_state & 0x1) << 0) & 0xFF
        raw_r_touch_switch_state = int(r_touch_switch_state)
        payload[1] |= ((raw_r_touch_switch_state & 0x1) << 5) & 0xFF
        raw_f_touch_switch_state = int(f_touch_switch_state)
        payload[1] |= ((raw_f_touch_switch_state & 0x1) << 4) & 0xFF
        raw_bms_state = int(bms_state)
        payload[1] |= ((raw_bms_state & 0x1) << 1) & 0xFF
        raw_emergency_button_state = int(emergency_button_state)
        payload[0] |= ((raw_emergency_button_state & 0x1) << 0) & 0xFF
        raw_parallel_state = int(parallel_state)
        payload[1] |= ((raw_parallel_state & 0x1) << 0) & 0xFF
        raw_dbs_state = int(dbs_state)
        payload[0] |= ((raw_dbs_state & 0x1) << 7) & 0xFF
        raw_ad_state = int(ad_state)
        payload[0] |= ((raw_ad_state & 0x1) << 6) & 0xFF
        raw_remote_state = int(remote_state)
        payload[0] |= ((raw_remote_state & 0x1) << 5) & 0xFF
        raw_motor_state = int(motor_state)
        payload[0] |= ((raw_motor_state & 0x1) << 1) & 0xFF
        return payload

    def _s1_api_remotet10_control_shake_2(
        self,
        remote_y1_longitudinal_control: float = 0.0,
        remote_x2_lateral_control: float = 0.0
    ) -> bytearray:
        """
        Build RemoteT10_Control_Shake_2 (CAN ID: 0x10b) payload bytearray.
        """
        payload = bytearray(8)
        raw_remote_y1_longitudinal_control = int(remote_y1_longitudinal_control)
        payload[4] = raw_remote_y1_longitudinal_control & 0xFF
        payload[5] = (raw_remote_y1_longitudinal_control >> 8) & 0xFF
        raw_remote_x2_lateral_control = int(remote_x2_lateral_control)
        payload[0] = raw_remote_x2_lateral_control & 0xFF
        payload[1] = (raw_remote_x2_lateral_control >> 8) & 0xFF
        return payload

    def _s1_api_remotet10_control_io(
        self,
        remote_f_horn: float = 0.0,
        remote_d_headlight: float = 0.0,
        remote_b_motor_holding_brake: float = 0.0,
        remote_a_mode_switch: float = 0.0,
        remote_c_speed_torque: float = 0.0,
        remote_e_gear: float = 0.0
    ) -> bytearray:
        """
        Build RemoteT10_Control_IO (CAN ID: 0x10a) payload bytearray.
        """
        payload = bytearray(8)
        raw_remote_f_horn = int(remote_f_horn)
        payload[1] = raw_remote_f_horn & 0xFF
        raw_remote_d_headlight = int(remote_d_headlight)
        payload[5] = raw_remote_d_headlight & 0xFF
        raw_remote_b_motor_holding_brake = int(remote_b_motor_holding_brake)
        payload[3] = raw_remote_b_motor_holding_brake & 0xFF
        raw_remote_a_mode_switch = int(remote_a_mode_switch)
        payload[2] = raw_remote_a_mode_switch & 0xFF
        raw_remote_c_speed_torque = int(remote_c_speed_torque)
        payload[4] = raw_remote_c_speed_torque & 0xFF
        raw_remote_e_gear = int(remote_e_gear)
        payload[0] = raw_remote_e_gear & 0xFF
        return payload

    def _s1_api_eps_status(
        self,
        eps_strangle_act: float = 0.0,
        eps_temperature: float = 0.0,
        eps_motor_current: float = 0.0,
        eps_fault: float = 0.0,
        eps_calibration_status: float = 0.0,
        eps_work_mode: float = 0.0
    ) -> bytearray:
        """
        Build EPS_Status (CAN ID: 0x18f) payload bytearray.
        """
        payload = bytearray(8)
        raw_eps_strangle_act = int(eps_strangle_act)
        payload[1] |= ((raw_eps_strangle_act & 0xFFFF) << 7) & 0xFF
        raw_eps_temperature = int(eps_temperature)
        payload[6] |= ((raw_eps_temperature & 0xFF) << 7) & 0xFF
        raw_eps_motor_current = int(eps_motor_current)
        payload[3] |= ((raw_eps_motor_current & 0xFFFF) << 7) & 0xFF
        raw_eps_fault = int(eps_fault)
        payload[0] |= ((raw_eps_fault & 0x1) << 1) & 0xFF
        raw_eps_calibration_status = int(eps_calibration_status)
        payload[0] |= ((raw_eps_calibration_status & 0x1) << 2) & 0xFF
        raw_eps_work_mode = int(eps_work_mode)
        payload[0] |= ((raw_eps_work_mode & 0x1) << 0) & 0xFF
        return payload

    def _s1_api_vcu_dbs_request(
        self,
        vcu_abs_active: float = 0.0,
        vcu_dbs_request_flag: float = 0.0,
        vcu_dbs_pressure_request: float = 0.0,
        vcu_dbs_work_mode: float = 0.0
    ) -> bytearray:
        """
        Build VCU_DBS_Request (CAN ID: 0x154) payload bytearray.
        """
        payload = bytearray(8)
        raw_vcu_abs_active = int(vcu_abs_active)
        payload[3] = raw_vcu_abs_active & 0xFF
        raw_vcu_dbs_request_flag = int(vcu_dbs_request_flag)
        payload[0] = raw_vcu_dbs_request_flag & 0xFF
        raw_vcu_dbs_pressure_request = int(round((vcu_dbs_pressure_request - (0.0)) / 0.1))
        payload[2] = raw_vcu_dbs_pressure_request & 0xFF
        raw_vcu_dbs_work_mode = int(vcu_dbs_work_mode)
        payload[1] = raw_vcu_dbs_work_mode & 0xFF
        return payload

    def _s1_api_dbs_status2(
        self,
        dbs_checksum: float = 0.0,
        dbs_rollingcounter: float = 0.0,
        dbs_waringcode: float = 0.0,
        dbs_faultcode: float = 0.0
    ) -> bytearray:
        """
        Build DBS_Status2 (CAN ID: 0x143) payload bytearray.
        """
        payload = bytearray(8)
        raw_dbs_checksum = int(dbs_checksum)
        payload[7] = raw_dbs_checksum & 0xFF
        raw_dbs_rollingcounter = int(dbs_rollingcounter)
        payload[6] |= ((raw_dbs_rollingcounter & 0xF) << 0) & 0xFF
        raw_dbs_waringcode = int(dbs_waringcode)
        payload[3] = raw_dbs_waringcode & 0xFF
        payload[4] = (raw_dbs_waringcode >> 8) & 0xFF
        payload[5] = (raw_dbs_waringcode >> 16) & 0xFF
        raw_dbs_faultcode = int(dbs_faultcode)
        payload[0] = raw_dbs_faultcode & 0xFF
        payload[1] = (raw_dbs_faultcode >> 8) & 0xFF
        payload[2] = (raw_dbs_faultcode >> 16) & 0xFF
        return payload

    def _s1_api_dbs_status(
        self,
        dbs_estopflag: float = 0.0,
        dbs_pedaiflag: float = 0.0,
        dbs_ref_iq: float = 0.0,
        dbs_work_mode: float = 0.0,
        brakepressurereqack: float = 0.0,
        dbs_rollingcounter: float = 0.0,
        dbs_park_warning: float = 0.0,
        dbs_peadalopening: float = 0.0,
        dbs_checksum: float = 0.0,
        dbs_hp_pressure: float = 0.0,
        dbs_system_status: float = 0.0
    ) -> bytearray:
        """
        Build DBS_Status (CAN ID: 0x142) payload bytearray.
        """
        payload = bytearray(8)
        raw_dbs_estopflag = int(dbs_estopflag)
        payload[6] |= ((raw_dbs_estopflag & 0x1) << 6) & 0xFF
        raw_dbs_pedaiflag = int(dbs_pedaiflag)
        payload[6] |= ((raw_dbs_pedaiflag & 0x1) << 7) & 0xFF
        raw_dbs_ref_iq = int(round((dbs_ref_iq - (-20.0)) / 0.5))
        payload[5] = raw_dbs_ref_iq & 0xFF
        raw_dbs_work_mode = int(dbs_work_mode)
        payload[1] = raw_dbs_work_mode & 0xFF
        raw_brakepressurereqack = int(brakepressurereqack)
        payload[2] = raw_brakepressurereqack & 0xFF
        raw_dbs_rollingcounter = int(dbs_rollingcounter)
        payload[6] |= ((raw_dbs_rollingcounter & 0xF) << 0) & 0xFF
        raw_dbs_park_warning = int(dbs_park_warning)
        payload[0] |= ((raw_dbs_park_warning & 0x3) << 6) & 0xFF
        raw_dbs_peadalopening = int(dbs_peadalopening)
        payload[4] = raw_dbs_peadalopening & 0xFF
        raw_dbs_checksum = int(dbs_checksum)
        payload[7] = raw_dbs_checksum & 0xFF
        raw_dbs_hp_pressure = int(round((dbs_hp_pressure - (0.0)) / 0.1))
        payload[3] = raw_dbs_hp_pressure & 0xFF
        raw_dbs_system_status = int(dbs_system_status)
        payload[0] |= ((raw_dbs_system_status & 0x3) << 0) & 0xFF
        return payload

    def _s1_api_mcu_drive_motor_feedback_msg(
        self,
        clamping_brake_status: float = 0.0,
        mcu_motor_error_grade: float = 0.0,
        motor_controltemp: float = 0.0,
        motor_idc: float = 0.0,
        motor_udc: float = 0.0
    ) -> bytearray:
        """
        Build MCU_Drive_Motor_Feedback_Msg (CAN ID: 0x60) payload bytearray.
        """
        payload = bytearray(8)
        raw_clamping_brake_status = int(clamping_brake_status)
        payload[5] |= ((raw_clamping_brake_status & 0x1) << 7) & 0xFF
        raw_mcu_motor_error_grade = int(mcu_motor_error_grade)
        payload[5] |= ((raw_mcu_motor_error_grade & 0x3) << 1) & 0xFF
        raw_motor_controltemp = int(round((motor_controltemp - (-50.0)) / 1.0))
        payload[4] = raw_motor_controltemp & 0xFF
        raw_motor_idc = int(round((motor_idc - (-1000.0)) / 0.1))
        payload[2] = raw_motor_idc & 0xFF
        payload[3] = (raw_motor_idc >> 8) & 0xFF
        raw_motor_udc = int(round((motor_udc - (0.0)) / 0.1))
        payload[0] = raw_motor_udc & 0xFF
        payload[1] = (raw_motor_udc >> 8) & 0xFF
        return payload

    def _s1_api_vcu_mcu_request(
        self,
        mcu_clamping_brake_req: float = 0.0,
        mcu_speed_req: float = 0.0,
        mcu_torque_req: float = 0.0,
        mcu_drivemode: float = 0.0,
        mcu_vcu_motor_request_valid: float = 0.0
    ) -> bytearray:
        """
        Build VCU_MCU_Request (CAN ID: 0x160) payload bytearray.
        """
        payload = bytearray(8)
        raw_mcu_clamping_brake_req = int(mcu_clamping_brake_req)
        payload[0] |= ((raw_mcu_clamping_brake_req & 0x1) << 3) & 0xFF
        raw_mcu_speed_req = int(round((mcu_speed_req - (-7000.0)) / 1.0))
        payload[3] = raw_mcu_speed_req & 0xFF
        payload[4] = (raw_mcu_speed_req >> 8) & 0xFF
        raw_mcu_torque_req = int(round((mcu_torque_req - (-1000.0)) / 0.1))
        payload[1] = raw_mcu_torque_req & 0xFF
        payload[2] = (raw_mcu_torque_req >> 8) & 0xFF
        raw_mcu_drivemode = int(mcu_drivemode)
        payload[0] |= ((raw_mcu_drivemode & 0x3) << 1) & 0xFF
        raw_mcu_vcu_motor_request_valid = int(mcu_vcu_motor_request_valid)
        payload[0] |= ((raw_mcu_vcu_motor_request_valid & 0x1) << 0) & 0xFF
        return payload

    def _s1_api_mcu_torque_feedback(
        self,
        mcu_errorcode: float = 0.0,
        mcu_motortemp: float = 0.0,
        mcu_current: float = 0.0,
        mcu_torque: float = 0.0,
        mcu_speed: float = 0.0,
        mcu_shift: float = 0.0
    ) -> bytearray:
        """
        Build MCU_Torque_Feedback (CAN ID: 0x10) payload bytearray.
        """
        payload = bytearray(8)
        raw_mcu_errorcode = int(mcu_errorcode)
        payload[7] = raw_mcu_errorcode & 0xFF
        raw_mcu_motortemp = int(round((mcu_motortemp - (-50.0)) / 1.0))
        payload[6] = raw_mcu_motortemp & 0xFF
        raw_mcu_current = int(mcu_current)
        payload[4] |= ((raw_mcu_current & 0xFFF) << 4) & 0xFF
        raw_mcu_torque = int(round((mcu_torque - (-1000.0)) / 0.1))
        payload[2] |= ((raw_mcu_torque & 0xFFFF) << 4) & 0xFF
        raw_mcu_speed = int(round((mcu_speed - (-100000.0)) / 1.0))
        payload[0] |= ((raw_mcu_speed & 0x3FFFF) << 2) & 0xFF
        raw_mcu_shift = int(mcu_shift)
        payload[0] |= ((raw_mcu_shift & 0x3) << 0) & 0xFF
        return payload

    def _s1_api_vcu_meter_req(
        self,
        vcu_meter_req_voltage: float = 0.0,
        vcu_meter_req_soc: float = 0.0,
        vcu_meter_req_ready: float = 0.0,
        vcu_meter_req_mileage: float = 0.0,
        vcu_meter_req_errorcode: float = 0.0,
        vcu_meter_req_current: float = 0.0,
        vcu_meter_req_charge_state: float = 0.0
    ) -> bytearray:
        """
        Build VCU_Meter_Req (CAN ID: 0x712) payload bytearray.
        """
        payload = bytearray(8)
        raw_vcu_meter_req_voltage = int(vcu_meter_req_voltage)
        payload[0] |= ((raw_vcu_meter_req_voltage & 0x3FF) << 0) & 0xFF
        raw_vcu_meter_req_soc = int(vcu_meter_req_soc)
        payload[2] |= ((raw_vcu_meter_req_soc & 0xFF) << 4) & 0xFF
        raw_vcu_meter_req_ready = int(vcu_meter_req_ready)
        payload[7] |= ((raw_vcu_meter_req_ready & 0x1) << 6) & 0xFF
        raw_vcu_meter_req_mileage = int(vcu_meter_req_mileage)
        payload[3] |= ((raw_vcu_meter_req_mileage & 0xFFFFF) << 4) & 0xFF
        raw_vcu_meter_req_errorcode = int(vcu_meter_req_errorcode)
        payload[6] |= ((raw_vcu_meter_req_errorcode & 0x3FF) << 4) & 0xFF
        raw_vcu_meter_req_current = int(round((vcu_meter_req_current - (-500.0)) / 1.0))
        payload[1] |= ((raw_vcu_meter_req_current & 0x3FF) << 2) & 0xFF
        raw_vcu_meter_req_charge_state = int(vcu_meter_req_charge_state)
        payload[7] |= ((raw_vcu_meter_req_charge_state & 0x1) << 7) & 0xFF
        return payload

    def _s1_api_vcu_icm_req(
        self,
        vcu_icm_req_msg5: float = 0.0,
        vcu_icm_req_msg4: float = 0.0,
        vcu_icm_req_msg3: float = 0.0,
        vcu_icm_req_msg2: float = 0.0,
        vcu_icm_req_msg1: float = 0.0
    ) -> bytearray:
        """
        Build VCU_ICM_Req (CAN ID: 0x612) payload bytearray.
        """
        payload = bytearray(8)
        raw_vcu_icm_req_msg5 = int(vcu_icm_req_msg5)
        payload[4] = raw_vcu_icm_req_msg5 & 0xFF
        payload[5] = (raw_vcu_icm_req_msg5 >> 8) & 0xFF
        payload[6] = (raw_vcu_icm_req_msg5 >> 16) & 0xFF
        payload[7] = (raw_vcu_icm_req_msg5 >> 24) & 0xFF
        raw_vcu_icm_req_msg4 = int(vcu_icm_req_msg4)
        payload[3] = raw_vcu_icm_req_msg4 & 0xFF
        raw_vcu_icm_req_msg3 = int(vcu_icm_req_msg3)
        payload[2] = raw_vcu_icm_req_msg3 & 0xFF
        raw_vcu_icm_req_msg2 = int(vcu_icm_req_msg2)
        payload[1] = raw_vcu_icm_req_msg2 & 0xFF
        raw_vcu_icm_req_msg1 = int(vcu_icm_req_msg1)
        payload[0] = raw_vcu_icm_req_msg1 & 0xFF
        return payload

    def _s1_api_bms_2b1h(
        self,
        bms_hvbatlowesttemcellnum: float = 0.0,
        bms_hvbathighesttemcellnum: float = 0.0,
        bms_hvbathighesttem: float = 0.0,
        bms_hvbatlowesttem: float = 0.0
    ) -> bytearray:
        """
        Build BMS_2B1h (CAN ID: 0x2b1) payload bytearray.
        """
        payload = bytearray(8)
        raw_bms_hvbatlowesttemcellnum = int(bms_hvbatlowesttemcellnum)
        payload[4] = raw_bms_hvbatlowesttemcellnum & 0xFF
        raw_bms_hvbathighesttemcellnum = int(bms_hvbathighesttemcellnum)
        payload[5] = raw_bms_hvbathighesttemcellnum & 0xFF
        raw_bms_hvbathighesttem = int(round((bms_hvbathighesttem - (-40.0)) / 1.0))
        payload[1] = raw_bms_hvbathighesttem & 0xFF
        raw_bms_hvbatlowesttem = int(round((bms_hvbatlowesttem - (-40.0)) / 1.0))
        payload[0] = raw_bms_hvbatlowesttem & 0xFF
        return payload

    def _s1_api_bms_a0h(
        self,
        bms_hvdisplaysoh: float = 0.0,
        bms_sys_flt: float = 0.0,
        bms_charge_stscc: float = 0.0,
        bms_charge_stscc2: float = 0.0,
        bms_sys_sts: float = 0.0,
        bms_hvbatsoc: float = 0.0,
        bms_hvbatvol: float = 0.0,
        bms_hvbatcrnt: float = 0.0
    ) -> bytearray:
        """
        Build BMS_A0h (CAN ID: 0xa0) payload bytearray.
        """
        payload = bytearray(8)
        raw_bms_hvdisplaysoh = int(bms_hvdisplaysoh)
        payload[7] = raw_bms_hvdisplaysoh & 0xFF
        raw_bms_sys_flt = int(bms_sys_flt)
        payload[6] = raw_bms_sys_flt & 0xFF
        raw_bms_charge_stscc = int(bms_charge_stscc)
        payload[5] |= ((raw_bms_charge_stscc & 0x1) << 4) & 0xFF
        raw_bms_charge_stscc2 = int(bms_charge_stscc2)
        payload[5] |= ((raw_bms_charge_stscc2 & 0x1) << 3) & 0xFF
        raw_bms_sys_sts = int(bms_sys_sts)
        payload[5] |= ((raw_bms_sys_sts & 0x7) << 0) & 0xFF
        raw_bms_hvbatsoc = int(round((bms_hvbatsoc - (0.0)) / 0.4))
        payload[4] = raw_bms_hvbatsoc & 0xFF
        raw_bms_hvbatvol = int(round((bms_hvbatvol - (0.0)) / 0.1))
        payload[2] = raw_bms_hvbatvol & 0xFF
        payload[3] = (raw_bms_hvbatvol >> 8) & 0xFF
        raw_bms_hvbatcrnt = int(round((bms_hvbatcrnt - (-1000.0)) / 0.1))
        payload[0] = raw_bms_hvbatcrnt & 0xFF
        payload[1] = (raw_bms_hvbatcrnt >> 8) & 0xFF
        return payload

    def _s1_api_bms_1a2h(
        self,
        charge_relay_sts: float = 0.0,
        acdc_relay_sts: float = 0.0,
        bms_weakupsig: float = 0.0,
        bms_keyon: float = 0.0,
        bms_req_hvdown: float = 0.0,
        bms_selfchk_sts: float = 0.0,
        heat_relay_sts: float = 0.0,
        precharge_sts: float = 0.0,
        charge_sts: float = 0.0,
        bms_heat_beat: float = 0.0,
        precharge_relay_sts: float = 0.0,
        parent_relay_sts: float = 0.0
    ) -> bytearray:
        """
        Build BMS_1A2h (CAN ID: 0x1a2) payload bytearray.
        """
        payload = bytearray(8)
        raw_charge_relay_sts = int(charge_relay_sts)
        payload[6] |= ((raw_charge_relay_sts & 0x1) << 3) & 0xFF
        raw_acdc_relay_sts = int(acdc_relay_sts)
        payload[6] |= ((raw_acdc_relay_sts & 0x1) << 4) & 0xFF
        raw_bms_weakupsig = int(bms_weakupsig)
        payload[0] |= ((raw_bms_weakupsig & 0xF) << 0) & 0xFF
        raw_bms_keyon = int(round((bms_keyon - (0.0)) / 0.1))
        payload[2] = raw_bms_keyon & 0xFF
        raw_bms_req_hvdown = int(bms_req_hvdown)
        payload[5] |= ((raw_bms_req_hvdown & 0x1) << 0) & 0xFF
        raw_bms_selfchk_sts = int(bms_selfchk_sts)
        payload[5] |= ((raw_bms_selfchk_sts & 0x7) << 3) & 0xFF
        raw_heat_relay_sts = int(heat_relay_sts)
        payload[5] |= ((raw_heat_relay_sts & 0x1) << 6) & 0xFF
        raw_precharge_sts = int(precharge_sts)
        payload[6] |= ((raw_precharge_sts & 0x3) << 1) & 0xFF
        raw_charge_sts = int(charge_sts)
        payload[6] |= ((raw_charge_sts & 0x7) << 5) & 0xFF
        raw_bms_heat_beat = int(bms_heat_beat)
        payload[7] |= ((raw_bms_heat_beat & 0xF) << 0) & 0xFF
        raw_precharge_relay_sts = int(precharge_relay_sts)
        payload[6] |= ((raw_precharge_relay_sts & 0x1) << 0) & 0xFF
        raw_parent_relay_sts = int(parent_relay_sts)
        payload[5] |= ((raw_parent_relay_sts & 0x1) << 7) & 0xFF
        return payload

    # Additional CAN0 DBC Messages

    def _s1_api_bms_1a3h(
        self,
        bms_chgpwr: float = 0.0,
        bms_chgpwr30s: float = 0.0,
        bms_pwr: float = 0.0,
        bms_pwr30s: float = 0.0
    ) -> bytearray:
        """
        Build BMS_1A3h (CAN ID: 0x1a3) payload bytearray from S1_CAN0_v5.dbc.
        """
        payload = bytearray(8)
        raw_bms_chgpwr = int(round((bms_chgpwr - (0)) / 0.1))
        payload[0] = raw_bms_chgpwr & 0xFF
        payload[1] = (raw_bms_chgpwr >> 8) & 0xFF
        raw_bms_chgpwr30s = int(round((bms_chgpwr30s - (0)) / 0.1))
        payload[2] = raw_bms_chgpwr30s & 0xFF
        payload[3] = (raw_bms_chgpwr30s >> 8) & 0xFF
        raw_bms_pwr = int(round((bms_pwr - (0)) / 0.1))
        payload[4] = raw_bms_pwr & 0xFF
        payload[5] = (raw_bms_pwr >> 8) & 0xFF
        raw_bms_pwr30s = int(round((bms_pwr30s - (0)) / 0.1))
        payload[6] = raw_bms_pwr30s & 0xFF
        payload[7] = (raw_bms_pwr30s >> 8) & 0xFF
        return payload

    def _s1_api_vcu_5a3h(
        self,
        vcu_poweron_bms: float = 0.0,
        vcu_ctrl_acdc: float = 0.0
    ) -> bytearray:
        """
        Build VCU_5A3h (CAN ID: 0x5a3) payload bytearray from S1_CAN0_v5.dbc.
        """
        payload = bytearray(8)
        raw_vcu_poweron_bms = int(vcu_poweron_bms)
        payload[0] = raw_vcu_poweron_bms & 0xFF
        raw_vcu_ctrl_acdc = int(vcu_ctrl_acdc)
        payload[1] = raw_vcu_ctrl_acdc & 0xFF
        return payload

    def _s1_api_bms_2b2h(
        self,
        bms_hvbatcelltem_no1: float = 0.0,
        bms_hvbatcelltem_no2: float = 0.0,
        bms_hvbatcelltem_no3: float = 0.0,
        bms_hvbatcelltem_no4: float = 0.0,
        bms_hvbatcelltem_no5: float = 0.0,
        bms_hvbatcelltem_no6: float = 0.0,
        bms_hvbatcelltem_no7: float = 0.0,
        bms_hvbatcelltem_no8: float = 0.0
    ) -> bytearray:
        """
        Build BMS_2B2h (CAN ID: 0x2b2) payload bytearray from S1_CAN0_v5.dbc.
        """
        payload = bytearray(8)
        raw_bms_hvbatcelltem_no1 = int(round((bms_hvbatcelltem_no1 - (-40)) / 1))
        payload[0] = raw_bms_hvbatcelltem_no1 & 0xFF
        raw_bms_hvbatcelltem_no2 = int(round((bms_hvbatcelltem_no2 - (-40)) / 1))
        payload[1] = raw_bms_hvbatcelltem_no2 & 0xFF
        raw_bms_hvbatcelltem_no3 = int(round((bms_hvbatcelltem_no3 - (-40)) / 1))
        payload[2] = raw_bms_hvbatcelltem_no3 & 0xFF
        raw_bms_hvbatcelltem_no4 = int(round((bms_hvbatcelltem_no4 - (-40)) / 1))
        payload[3] = raw_bms_hvbatcelltem_no4 & 0xFF
        raw_bms_hvbatcelltem_no5 = int(round((bms_hvbatcelltem_no5 - (-40)) / 1))
        payload[4] = raw_bms_hvbatcelltem_no5 & 0xFF
        raw_bms_hvbatcelltem_no6 = int(round((bms_hvbatcelltem_no6 - (-40)) / 1))
        payload[5] = raw_bms_hvbatcelltem_no6 & 0xFF
        raw_bms_hvbatcelltem_no7 = int(round((bms_hvbatcelltem_no7 - (-40)) / 1))
        payload[6] = raw_bms_hvbatcelltem_no7 & 0xFF
        raw_bms_hvbatcelltem_no8 = int(round((bms_hvbatcelltem_no8 - (-40)) / 1))
        payload[7] = raw_bms_hvbatcelltem_no8 & 0xFF
        return payload

    def _s1_api_bms_2a5h(
        self,
        bms_hvbatcellvol_no13: float = 0.0,
        bms_hvbatcellvol_no14: float = 0.0,
        bms_hvbatcellvol_no15: float = 0.0,
        bms_hvbatcellvol_no16: float = 0.0
    ) -> bytearray:
        """
        Build BMS_2A5h (CAN ID: 0x2a5) payload bytearray from S1_CAN0_v5.dbc.
        """
        payload = bytearray(8)
        raw_bms_hvbatcellvol_no13 = int(bms_hvbatcellvol_no13)
        payload[0] = raw_bms_hvbatcellvol_no13 & 0xFF
        payload[1] = (raw_bms_hvbatcellvol_no13 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no14 = int(bms_hvbatcellvol_no14)
        payload[2] = raw_bms_hvbatcellvol_no14 & 0xFF
        payload[3] = (raw_bms_hvbatcellvol_no14 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no15 = int(bms_hvbatcellvol_no15)
        payload[4] = raw_bms_hvbatcellvol_no15 & 0xFF
        payload[5] = (raw_bms_hvbatcellvol_no15 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no16 = int(bms_hvbatcellvol_no16)
        payload[6] = raw_bms_hvbatcellvol_no16 & 0xFF
        payload[7] = (raw_bms_hvbatcellvol_no16 >> 8) & 0xFF
        return payload

    def _s1_api_bms_2a4h(
        self,
        bms_hvbatcellvol_no9: float = 0.0,
        bms_hvbatcellvol_no10: float = 0.0,
        bms_hvbatcellvol_no11: float = 0.0,
        bms_hvbatcellvol_no12: float = 0.0
    ) -> bytearray:
        """
        Build BMS_2A4h (CAN ID: 0x2a4) payload bytearray from S1_CAN0_v5.dbc.
        """
        payload = bytearray(8)
        raw_bms_hvbatcellvol_no9 = int(bms_hvbatcellvol_no9)
        payload[0] = raw_bms_hvbatcellvol_no9 & 0xFF
        payload[1] = (raw_bms_hvbatcellvol_no9 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no10 = int(bms_hvbatcellvol_no10)
        payload[2] = raw_bms_hvbatcellvol_no10 & 0xFF
        payload[3] = (raw_bms_hvbatcellvol_no10 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no11 = int(bms_hvbatcellvol_no11)
        payload[4] = raw_bms_hvbatcellvol_no11 & 0xFF
        payload[5] = (raw_bms_hvbatcellvol_no11 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no12 = int(bms_hvbatcellvol_no12)
        payload[6] = raw_bms_hvbatcellvol_no12 & 0xFF
        payload[7] = (raw_bms_hvbatcellvol_no12 >> 8) & 0xFF
        return payload

    def _s1_api_bms_2a3h(
        self,
        bms_hvbatcellvol_no5: float = 0.0,
        bms_hvbatcellvol_no6: float = 0.0,
        bms_hvbatcellvol_no7: float = 0.0,
        bms_hvbatcellvol_no8: float = 0.0
    ) -> bytearray:
        """
        Build BMS_2A3h (CAN ID: 0x2a3) payload bytearray from S1_CAN0_v5.dbc.
        """
        payload = bytearray(8)
        raw_bms_hvbatcellvol_no5 = int(bms_hvbatcellvol_no5)
        payload[0] = raw_bms_hvbatcellvol_no5 & 0xFF
        payload[1] = (raw_bms_hvbatcellvol_no5 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no6 = int(bms_hvbatcellvol_no6)
        payload[2] = raw_bms_hvbatcellvol_no6 & 0xFF
        payload[3] = (raw_bms_hvbatcellvol_no6 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no7 = int(bms_hvbatcellvol_no7)
        payload[4] = raw_bms_hvbatcellvol_no7 & 0xFF
        payload[5] = (raw_bms_hvbatcellvol_no7 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no8 = int(bms_hvbatcellvol_no8)
        payload[6] = raw_bms_hvbatcellvol_no8 & 0xFF
        payload[7] = (raw_bms_hvbatcellvol_no8 >> 8) & 0xFF
        return payload

    def _s1_api_bms_2a2h(
        self,
        bms_hvbatcellvol_no1: float = 0.0,
        bms_hvbatcellvol_no2: float = 0.0,
        bms_hvbatcellvol_no3: float = 0.0,
        bms_hvbatcellvol_no4: float = 0.0
    ) -> bytearray:
        """
        Build BMS_2A2h (CAN ID: 0x2a2) payload bytearray from S1_CAN0_v5.dbc.
        """
        payload = bytearray(8)
        raw_bms_hvbatcellvol_no1 = int(bms_hvbatcellvol_no1)
        payload[0] = raw_bms_hvbatcellvol_no1 & 0xFF
        payload[1] = (raw_bms_hvbatcellvol_no1 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no2 = int(bms_hvbatcellvol_no2)
        payload[2] = raw_bms_hvbatcellvol_no2 & 0xFF
        payload[3] = (raw_bms_hvbatcellvol_no2 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no3 = int(bms_hvbatcellvol_no3)
        payload[4] = raw_bms_hvbatcellvol_no3 & 0xFF
        payload[5] = (raw_bms_hvbatcellvol_no3 >> 8) & 0xFF
        raw_bms_hvbatcellvol_no4 = int(bms_hvbatcellvol_no4)
        payload[6] = raw_bms_hvbatcellvol_no4 & 0xFF
        payload[7] = (raw_bms_hvbatcellvol_no4 >> 8) & 0xFF
        return payload

    def _s1_api_bms_2a1h(
        self,
        bms_hvbatlowestcellvol: float = 0.0,
        bms_hvbathighestcellvol: float = 0.0,
        bms_hvbatlowestvolcellnum: float = 0.0,
        bms_hvbathighestvolcellnum: float = 0.0
    ) -> bytearray:
        """
        Build BMS_2A1h (CAN ID: 0x2a1) payload bytearray from S1_CAN0_v5.dbc.
        """
        payload = bytearray(8)
        raw_bms_hvbatlowestcellvol = int(bms_hvbatlowestcellvol)
        payload[0] = raw_bms_hvbatlowestcellvol & 0xFF
        payload[1] = (raw_bms_hvbatlowestcellvol >> 8) & 0xFF
        raw_bms_hvbathighestcellvol = int(bms_hvbathighestcellvol)
        payload[2] = raw_bms_hvbathighestcellvol & 0xFF
        payload[3] = (raw_bms_hvbathighestcellvol >> 8) & 0xFF
        raw_bms_hvbatlowestvolcellnum = int(bms_hvbatlowestvolcellnum)
        payload[4] = raw_bms_hvbatlowestvolcellnum & 0xFF
        raw_bms_hvbathighestvolcellnum = int(bms_hvbathighestvolcellnum)
        payload[5] = raw_bms_hvbathighestvolcellnum & 0xFF
        return payload
