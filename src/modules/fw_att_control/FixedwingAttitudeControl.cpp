/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "FixedwingAttitudeControl.hpp"

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);

FixedwingAttitudeControl::FixedwingAttitudeControl() :
	_airspeed_sub(ORB_ID(airspeed)),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fwa_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fwa_nano"))
{
	_parameter_handles.p_tc = param_find("FW_P_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");

	_parameter_handles.r_tc = param_find("FW_R_TC");
	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");
	_parameter_handles.roll_to_yaw_ff = param_find("FW_RLL_TO_YAW_FF");

	_parameter_handles.w_en = param_find("FW_W_EN");
	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.dtrim_roll_vmin = param_find("FW_DTRIM_R_VMIN");
	_parameter_handles.dtrim_pitch_vmin = param_find("FW_DTRIM_P_VMIN");
	_parameter_handles.dtrim_yaw_vmin = param_find("FW_DTRIM_Y_VMIN");
	_parameter_handles.dtrim_roll_vmax = param_find("FW_DTRIM_R_VMAX");
	_parameter_handles.dtrim_pitch_vmax = param_find("FW_DTRIM_P_VMAX");
	_parameter_handles.dtrim_yaw_vmax = param_find("FW_DTRIM_Y_VMAX");
	_parameter_handles.dtrim_roll_flaps = param_find("FW_DTRIM_R_FLPS");
	_parameter_handles.dtrim_pitch_flaps = param_find("FW_DTRIM_P_FLPS");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");
	_parameter_handles.man_roll_scale = param_find("FW_MAN_R_SC");
	_parameter_handles.man_pitch_scale = param_find("FW_MAN_P_SC");
	_parameter_handles.man_yaw_scale = param_find("FW_MAN_Y_SC");

	_parameter_handles.acro_max_x_rate = param_find("FW_ACRO_X_MAX");
	_parameter_handles.acro_max_y_rate = param_find("FW_ACRO_Y_MAX");
	_parameter_handles.acro_max_z_rate = param_find("FW_ACRO_Z_MAX");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaps_takeoff_scale = param_find("FW_FLAPS_TO_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.rattitude_thres = param_find("FW_RATT_TH");

	_parameter_handles.bat_scale_en = param_find("FW_BAT_SCALE_EN");
	_parameter_handles.airspeed_mode = param_find("FW_ARSP_MODE");

    /// ========> ////////////////////////////////////////////////////////////////////
    // Etienne - Suwave custom parameters for vertical aquatic takeoff
    _parameter_handles.take_off_custom_time_01 = param_find("TK_WAIT_TIME");
    _parameter_handles.take_off_custom_time_03 = param_find("TK_RISE_TIME");
    _parameter_handles.take_off_custom_time_04 = param_find("TK_CLIMB_TIME");
	_parameter_handles.take_off_prop_fly= param_find("TK_PROP_FLY");
    _parameter_handles.take_off_prop_horizontal= param_find("TK_PROP_HORI");
    _parameter_handles.take_off_prop_vertical= param_find("TK_PROP_VERT");
    _parameter_handles.take_off_rudder_offset= param_find("TK_RUDDER_OFF");
    _parameter_handles.take_off_rising_pitch_des= param_find("TK_RISE_PITCH_DE");
    _parameter_handles.take_off_rising_pitch_kp= param_find("TK_RISE_PITCH_KP");
    _parameter_handles.take_off_rising_pitch_kd= param_find("TK_RISE_PITCH_KD");
    _parameter_handles.take_off_rising_yaw_kp= param_find("TK_RISE_YAW_KP");
    _parameter_handles.take_off_rising_yaw_kd= param_find("TK_RISE_YAW_KD");
    _parameter_handles.take_off_climbing_pitch_des= param_find("TK_CLB_PITCH_DES");
    _parameter_handles.take_off_climbing_pitch_kp= param_find("TK_CLB_PITCH_KP");
    _parameter_handles.take_off_climbing_pitch_kd= param_find("TK_CLB_PITCH_KD");
    _parameter_handles.take_off_climbing_elev_kp= param_find("TK_CLB_ELEV_KP");
    _parameter_handles.take_off_climbing_elev_kd= param_find("TK_CLB_ELEV_KD");
    _parameter_handles.take_off_climbing_roll_kp= param_find("TK_CLB_ROLL_KP");
    _parameter_handles.take_off_climbing_roll_kd= param_find("TK_CLB_ROLL_KD");
    _parameter_handles.take_off_climbing_yawrate_kp= param_find("TK_CLB_YAWRAT_KP");
    _parameter_handles.take_off_indoor= param_find("TK_INDOOR");
    _parameter_handles.take_off_height_agl_trigger= param_find("TK_AGL_TRIG");

    /// <======= ////////////////////////////////////////////////////////////////////

	// initialize to invalid VTOL type
	_parameters.vtol_type = -1;

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);
}

int
FixedwingAttitudeControl::parameters_update()
{

	int32_t tmp = 0;
	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));
	param_get(_parameter_handles.roll_to_yaw_ff, &(_parameters.roll_to_yaw_ff));

	param_get(_parameter_handles.w_en, &tmp);
	_parameters.w_en = (tmp == 1);

	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.dtrim_roll_vmin, &(_parameters.dtrim_roll_vmin));
	param_get(_parameter_handles.dtrim_roll_vmax, &(_parameters.dtrim_roll_vmax));
	param_get(_parameter_handles.dtrim_pitch_vmin, &(_parameters.dtrim_pitch_vmin));
	param_get(_parameter_handles.dtrim_pitch_vmax, &(_parameters.dtrim_pitch_vmax));
	param_get(_parameter_handles.dtrim_yaw_vmin, &(_parameters.dtrim_yaw_vmin));
	param_get(_parameter_handles.dtrim_yaw_vmax, &(_parameters.dtrim_yaw_vmax));

	param_get(_parameter_handles.dtrim_roll_flaps, &(_parameters.dtrim_roll_flaps));
	param_get(_parameter_handles.dtrim_pitch_flaps, &(_parameters.dtrim_pitch_flaps));

	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);
	param_get(_parameter_handles.man_roll_scale, &(_parameters.man_roll_scale));
	param_get(_parameter_handles.man_pitch_scale, &(_parameters.man_pitch_scale));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));

	param_get(_parameter_handles.acro_max_x_rate, &(_parameters.acro_max_x_rate_rad));
	param_get(_parameter_handles.acro_max_y_rate, &(_parameters.acro_max_y_rate_rad));
	param_get(_parameter_handles.acro_max_z_rate, &(_parameters.acro_max_z_rate_rad));
	_parameters.acro_max_x_rate_rad = math::radians(_parameters.acro_max_x_rate_rad);
	_parameters.acro_max_y_rate_rad = math::radians(_parameters.acro_max_y_rate_rad);
	_parameters.acro_max_z_rate_rad = math::radians(_parameters.acro_max_z_rate_rad);

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaps_takeoff_scale, &_parameters.flaps_takeoff_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.rattitude_thres, &_parameters.rattitude_thres);

    /// ========> ////////////////////////////////////////////////////////////////////
    // Etienne - Suwave custom parameters for vertical aquatic takeoff
    param_get(_parameter_handles.take_off_custom_time_01, &_parameters.take_off_custom_time_01);
    param_get(_parameter_handles.take_off_custom_time_03, &_parameters.take_off_custom_time_03);
    param_get(_parameter_handles.take_off_custom_time_04, &_parameters.take_off_custom_time_04);
	param_get(_parameter_handles.take_off_prop_fly, &_parameters.take_off_prop_fly);
    param_get(_parameter_handles.take_off_prop_horizontal, &_parameters.take_off_prop_horizontal);
    param_get(_parameter_handles.take_off_prop_vertical, &_parameters.take_off_prop_vertical);
    param_get(_parameter_handles.take_off_rudder_offset, &_parameters.take_off_rudder_offset);
    param_get(_parameter_handles.take_off_rising_pitch_des, &_parameters.take_off_rising_pitch_des);
    param_get(_parameter_handles.take_off_rising_pitch_kp, &_parameters.take_off_rising_pitch_kp);
    param_get(_parameter_handles.take_off_rising_pitch_kd, &_parameters.take_off_rising_pitch_kd);
    param_get(_parameter_handles.take_off_rising_yaw_kp, &_parameters.take_off_rising_yaw_kp);
    param_get(_parameter_handles.take_off_rising_yaw_kd, &_parameters.take_off_rising_yaw_kd);
    param_get(_parameter_handles.take_off_climbing_pitch_des, &_parameters.take_off_climbing_pitch_des);
    param_get(_parameter_handles.take_off_climbing_pitch_kp, &_parameters.take_off_climbing_pitch_kp);
    param_get(_parameter_handles.take_off_climbing_pitch_kd, &_parameters.take_off_climbing_pitch_kd);
    param_get(_parameter_handles.take_off_climbing_elev_kp, &_parameters.take_off_climbing_elev_kp);
    param_get(_parameter_handles.take_off_climbing_elev_kd, &_parameters.take_off_climbing_elev_kd);
    param_get(_parameter_handles.take_off_climbing_roll_kp, &_parameters.take_off_climbing_roll_kp);
    param_get(_parameter_handles.take_off_climbing_roll_kd, &_parameters.take_off_climbing_roll_kd);
    param_get(_parameter_handles.take_off_climbing_yawrate_kp, &_parameters.take_off_climbing_yawrate_kp);
    param_get(_parameter_handles.take_off_indoor, &_parameters.take_off_indoor);
    param_get(_parameter_handles.take_off_height_agl_trigger, &_parameters.take_off_height_agl_trigger);

    /// <======= ////////////////////////////////////////////////////////////////////

	if (_vehicle_status.is_vtol) {
		param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);
	}

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

	param_get(_parameter_handles.airspeed_mode, &tmp);
	_parameters.airspeed_disabled = (tmp == 1);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_parameters.w_p);
	_wheel_ctrl.set_k_i(_parameters.w_i);
	_wheel_ctrl.set_k_ff(_parameters.w_ff);
	_wheel_ctrl.set_integrator_max(_parameters.w_integrator_max);
	_wheel_ctrl.set_max_rate(math::radians(_parameters.w_rmax));

	return PX4_OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	// only update manual if in a manual mode
	if (_vcontrol_mode.flag_control_manual_enabled) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual) == PX4_OK) {

			// Check if we are in rattitude mode and the pilot is above the threshold on pitch
			if (_vcontrol_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual.y) > _parameters.rattitude_thres || fabsf(_manual.x) > _parameters.rattitude_thres) {
					_vcontrol_mode.flag_control_attitude_enabled = false;
				}
			}

			if (!_vcontrol_mode.flag_control_climb_rate_enabled &&
			    !_vcontrol_mode.flag_control_offboard_enabled) {

				if (_vcontrol_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs
					_att_sp.timestamp = hrt_absolute_time();
					_att_sp.roll_body = _manual.y * _parameters.man_roll_max + _parameters.rollsp_offset_rad;
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, -_parameters.man_roll_max, _parameters.man_roll_max);
					_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max + _parameters.pitchsp_offset_rad;
					_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max, _parameters.man_pitch_max);
					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust = _manual.z;

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);
					_att_sp.q_d_valid = true;

					if (_attitude_sp_pub != nullptr) {
						/* publish the attitude rates setpoint */
						orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

					} else if (_attitude_setpoint_id) {
						/* advertise the attitude rates setpoint */
						_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
					}

				} else if (_vcontrol_mode.flag_control_rates_enabled &&
					   !_vcontrol_mode.flag_control_attitude_enabled) {

					// RATE mode we need to generate the rate setpoint from manual user inputs
					_rates_sp.timestamp = hrt_absolute_time();
					_rates_sp.roll = _manual.y * _parameters.acro_max_x_rate_rad;
					_rates_sp.pitch = -_manual.x * _parameters.acro_max_y_rate_rad;
					_rates_sp.yaw = _manual.r * _parameters.acro_max_z_rate_rad;
					_rates_sp.thrust = _manual.z;

					if (_rate_sp_pub != nullptr) {
						/* publish the attitude rates setpoint */
						orb_publish(_rates_sp_id, _rate_sp_pub, &_rates_sp);

					} else if (_rates_sp_id) {
						/* advertise the attitude rates setpoint */
						_rate_sp_pub = orb_advertise(_rates_sp_id, &_rates_sp);
					}

				} else {
					/* manual/direct control */
					_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _parameters.man_roll_scale + _parameters.trim_roll;
					_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale +
							_parameters.trim_pitch;
					_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
                    _actuators_airframe.control[1] = _parameters.take_off_prop_fly;
                    _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;
				}
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}
}

void
FixedwingAttitudeControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedwingAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

				_parameter_handles.vtol_type = param_find("VT_TYPE");

				parameters_update();

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////====================================>
/* As per Gabriel Guilmain's work from summer 2017, the aquatic custom takeoff controller is insert here.
 * Etienne at winter 2019 reorganise the aquatic custom takeoff controller in Switch sequence in its own function*/


void
FixedwingAttitudeControl::vertical_takeoff_controller() {
	_verticalTk.qAtt = _att.q;
	_verticalTk.eulAtt = _verticalTk.qAtt;

    /* only for debug */
//    static int _countPrint =0;
//    if (++_countPrint >= 100)
//    {
//        warn("_pitchErr : %0.3f", (double)(_pitchErr*R2D));
//        warn("_rollErr  : %0.3f", (double)(_rollErr*R2D));
//        _countPrint = 0;
//    }

    /* Sequences of the controller for the custom takeoff */
    float r2servo = (_parameters.take_off_prop_vertical - _parameters.take_off_prop_horizontal) / (3.14159f / 2);
    float _elevDes;
	float _pitchErr;
	float _rollErr;
	float _yawErr;

    switch (mode_seq) {
        // 1 - WAIT AVANT LA SEQUENCE (FALCULTATIF)
        case WAIT :
            _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
            _actuators_airframe.control[1] = _parameters.take_off_prop_fly; //0.28f;
            _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;
            _actuators.control[actuator_controls_s::INDEX_ROLL] = _parameters.trim_roll;
            _actuators.control[actuator_controls_s::INDEX_PITCH] = _parameters.trim_pitch;


            if (hrt_absolute_time() - present_time >= _parameters.take_off_custom_time_01) //
            {
                warnx("Etienne Start");
                present_time = hrt_absolute_time();
                mode_seq = FLIP;
                _verticalTk.alt0 = _local_pos.z;
                _verticalTk.head0 = _verticalTk.eulAtt(2);

            }
            break;

        // 2 - ACTIVE LE SERVO POUR REMONTER LE PIVOT
        case FLIP :
            _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
            _actuators_airframe.control[1] = _parameters.take_off_prop_vertical;
            _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;
            _actuators.control[actuator_controls_s::INDEX_ROLL] = _parameters.trim_roll;
            _actuators.control[actuator_controls_s::INDEX_PITCH] = _parameters.trim_pitch;
            _verticalTk.alt0 = EMACOEF*_local_pos.z + (1-EMACOEF) * _verticalTk.alt0;

            if (hrt_absolute_time() - present_time >= 1000000) //(int)_parameters.take_off_custom_time_03) // 1 sec
            {
                warnx("Transit to TakeOff Control");
                present_time = hrt_absolute_time();
                mode_seq = RISING;
                warnx("_verticalTk.alt0  : %0.3f", (double)(_verticalTk.alt0));
                warnx("_verticalTk.head0 : %0.3f", (double)(_verticalTk.head0*R2D));
            }
            break;


        // 3 - Vertical takeoff Control, Rising sequence
        case RISING :
            _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
			_verticalTk.eulDes = Eulerf(0.0f, _parameters.take_off_rising_pitch_des*D2R, _verticalTk.head0);
            _verticalTk.qDes = Quatf(_verticalTk.eulDes);
            _verticalTk.qAtt2Des = _verticalTk.qAtt.inversed() * _verticalTk.qDes;
            _verticalTk.eulAtt2Des = _verticalTk.qAtt2Des;

			// Conversion du quaternion d'erreur en angle d'erreur via une transformée YZX pour permettre un pitchErr > 90
			_pitchErr = atan2f(-2.0f * (_verticalTk.qAtt2Des(1) * _verticalTk.qAtt2Des(3) - _verticalTk.qAtt2Des(0) * _verticalTk.qAtt2Des(2)),
									 1.0f - 2.0f * (_verticalTk.qAtt2Des(2) * _verticalTk.qAtt2Des(2) + _verticalTk.qAtt2Des(3) * _verticalTk.qAtt2Des(3)));
			_yawErr = asinf(2.0f * (_verticalTk.qAtt2Des(1) * _verticalTk.qAtt2Des(2) + _verticalTk.qAtt2Des(0) * _verticalTk.qAtt2Des(3)));
            _rollErr = atan2f(-2.0f * (_verticalTk.qAtt2Des(2) * _verticalTk.qAtt2Des(3) - _verticalTk.qAtt2Des(0) * _verticalTk.qAtt2Des(1)),
                              1.0f - 2.0f * (_verticalTk.qAtt2Des(1) * _verticalTk.qAtt2Des(1) + _verticalTk.qAtt2Des(3) * _verticalTk.qAtt2Des(3)));

			_actuators_airframe.control[1] = (_parameters.take_off_rising_pitch_kp * _pitchErr -
                                              _parameters.take_off_rising_pitch_kd * _att.pitchspeed) * r2servo +
                                             _parameters.take_off_prop_horizontal;
            _actuators_airframe.control[2] = (_parameters.take_off_rising_yaw_kp * _yawErr -
                                              _parameters.take_off_rising_yaw_kd * _att.yawspeed) +
                                             _parameters.take_off_rudder_offset;
            _actuators.control[actuator_controls_s::INDEX_ROLL] = _rollErr * _parameters.take_off_climbing_roll_kp
                                                                  - _att.rollspeed * _parameters.take_off_climbing_roll_kd
                                                                  + _parameters.trim_roll;
            _actuators.control[actuator_controls_s::INDEX_PITCH] = _parameters.trim_pitch;



            if (hrt_absolute_time() - present_time >= _parameters.take_off_custom_time_03) // 2 sec
            {
                warnx("Transit to NoseDown Control");
                present_time = hrt_absolute_time();
                mode_seq = CLIMBING;
                warnx("_local_pos.z : %0.3f", (double)(_local_pos.z));
            }
            break;

        // 4 - NoseDown Control, Climbing sequence
        case CLIMBING :
			// Present attitude from Quaternion to Euler ZXY
            float t = (hrt_absolute_time() - present_time);
            float decayFilter = (expf(-5.0f*t/_parameters.take_off_custom_time_04));

            _elevDes = ((_parameters.take_off_rising_pitch_des -_parameters.take_off_climbing_pitch_des)*decayFilter +_parameters.take_off_climbing_pitch_des) * D2R;

            float _headingNow = atan2f(-2.0f * (_verticalTk.qAtt(1) * _verticalTk.qAtt(2) - _verticalTk.qAtt(0) * _verticalTk.qAtt(3)),
                                       1.0f - 2.0f * (_verticalTk.qAtt(1) * _verticalTk.qAtt(1) + _verticalTk.qAtt(3) * _verticalTk.qAtt(3)));
            float _bankNow = asinf(2.0f * (_verticalTk.qAtt(2) * _verticalTk.qAtt(3) + _verticalTk.qAtt(0) * _verticalTk.qAtt(1)));

            // Quaternion with the right heading and elevation from nose down movement of present attitude - From Euler Rotation ZXY to Quaternion
			float cang[3] = {cosf(_headingNow / 2.0f), cosf(_bankNow / 2.0f), cosf(_elevDes / cosf(_bankNow) / 2.0f)};
            float sang[3] = {sinf(_headingNow / 2.0f), sinf(_bankNow / 2.0f), sinf(_elevDes / cosf(_bankNow) / 2.0f)};
            matrix::Quatf _qElev;
            _qElev(0) = cang[0] * cang[1] * cang[2] - sang[0] * sang[1] * sang[2];
            _qElev(1) = cang[0] * sang[1] * cang[2] - sang[0] * cang[1] * sang[2];
            _qElev(2) = cang[0] * cang[1] * sang[2] + sang[0] * sang[1] * cang[2];
            _qElev(3) = cang[0] * sang[1] * sang[2] + sang[0] * cang[1] * cang[2];
            _verticalTk.qAtt2Des = _verticalTk.qAtt.inversed() * _qElev;

            // Euler angle error from Quaternion error - Rotation YXZ to exclude yaw movement as required by the error calculation and allow pitch movement >90°
            _rollErr = -_bankNow;
            _pitchErr = atan2f(2.0f * (_verticalTk.qAtt2Des(1) * _verticalTk.qAtt2Des(3) + _verticalTk.qAtt2Des(0) * _verticalTk.qAtt2Des(2)),
                                     1.0f - 2.0f * (_verticalTk.qAtt2Des(1) * _verticalTk.qAtt2Des(1) + _verticalTk.qAtt2Des(2) * _verticalTk.qAtt2Des(2)));


            if (_parameters.take_off_indoor) {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f * decayFilter;
            }
            else {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
            }

            _actuators_airframe.control[1] = (_pitchErr * _parameters.take_off_climbing_pitch_kp -
                                              _att.pitchspeed * _parameters.take_off_climbing_pitch_kd) * r2servo +
                                             _parameters.take_off_prop_horizontal;
            _actuators_airframe.control[2] =
                    (-_att.yawspeed * _parameters.take_off_climbing_yawrate_kp) + _parameters.take_off_rudder_offset;
            _actuators.control[actuator_controls_s::INDEX_ROLL] = _rollErr * _parameters.take_off_climbing_roll_kp
                                                                  - _att.rollspeed * _parameters.take_off_climbing_roll_kd
                                                                  + _parameters.trim_roll;
            _actuators.control[actuator_controls_s::INDEX_PITCH ] = (_pitchErr * _parameters.take_off_climbing_elev_kp)*signbit(-_pitchErr)
                                                                     - (_att.pitchspeed * _parameters.take_off_climbing_elev_kd)*signbit(_att.pitchspeed)
                                                                     + _parameters.trim_pitch;

            if (hrt_absolute_time() - present_time >= _parameters.take_off_custom_time_04) // 120 ms
            {
                warnx("Transit to Px4 Control");
                present_time = hrt_absolute_time();
                mode_seq = WAIT;
                mode_take_off_custom = false;
            }
            break;
    }
}

////<======================///////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &vehicle_land_detected) == PX4_OK) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

void FixedwingAttitudeControl::run()
{
	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	vehicle_land_detected_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	/* Setup of loop */
	fds[0].fd = _att_sub;
	fds[0].events = POLLIN;

    /////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////====================================>

    /* As per Gabriel Guilmain's work from summer 2017, the aquatic custom takeoff controller is insert here */
    // Etienne et Étienne

    mode_take_off_custom = false;
    take_off_trigger = false;
    present_time = hrt_absolute_time(); // timer pour les etapes du decollage
    mode_seq = WAIT;


    ////<======================///////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

	while (!should_exit()) {

		/* only update parameters if they changed */
		bool params_updated = false;
		orb_check(_params_sub, &params_updated);

		if (params_updated) {
			/* read from param to clear updated flag */
			parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only run controller if attitude changed */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
            orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

			/* get current rotation matrix and euler angles from control state quaternions */
			matrix::Dcmf R = matrix::Quatf(_att.q);

			if (_vehicle_status.is_vtol && _parameters.vtol_type == vtol_type::TAILSITTER) {
				/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
				 *
				 * Since the VTOL airframe is initialized as a multicopter we need to
				 * modify the estimated attitude for the fixed wing operation.
				 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
				 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
				 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
				 * Additionally, in order to get the correct sign of the pitch, we need to multiply
				 * the new x axis of the rotation matrix with -1
				 *
				 * original:			modified:
				 *
				 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
				 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
				 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
				 * */
				matrix::Dcmf R_adapted = R;		//modified rotation matrix

				/* move z to x */
				R_adapted(0, 0) = R(0, 2);
				R_adapted(1, 0) = R(1, 2);
				R_adapted(2, 0) = R(2, 2);

				/* move x to z */
				R_adapted(0, 2) = R(0, 0);
				R_adapted(1, 2) = R(1, 0);
				R_adapted(2, 2) = R(2, 0);

				/* change direction of pitch (convert to right handed system) */
				R_adapted(0, 0) = -R_adapted(0, 0);
				R_adapted(1, 0) = -R_adapted(1, 0);
				R_adapted(2, 0) = -R_adapted(2, 0);

				/* fill in new attitude data */
				R = R_adapted;

				/* lastly, roll- and yawspeed have to be swaped */
				float helper = _att.rollspeed;
				_att.rollspeed = -_att.yawspeed;
				_att.yawspeed = helper;
			}

			matrix::Eulerf euler_angles(R);

			_airspeed_sub.update();
			vehicle_setpoint_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			global_pos_poll();
			vehicle_status_poll();
			vehicle_land_detected_poll();

			// the position controller will not emit attitude setpoints in some modes
			// we need to make sure that this flag is reset
			_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

			/* lock integrator until control is started */
			bool lock_integrator = !(_vcontrol_mode.flag_control_rates_enabled && !_vehicle_status.is_rotary_wing);

			/* Simple handling of failsafe: deploy parachute if failsafe is on */
			if (_vcontrol_mode.flag_control_termination_enabled) {
				_actuators_airframe.control[7] = 1.0f;

			} else {
				_actuators_airframe.control[7] = 0.0f;
			}

			/* if we are in rotary wing mode, do nothing */
			if (_vehicle_status.is_rotary_wing && !_vehicle_status.is_vtol) {
				continue;
			}

			control_flaps(deltaT);

			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_rates_enabled) {
				/* scale around tuning airspeed */
				float airspeed;

				/* if airspeed is non-finite or not valid or if we are asked not to control it, we assume the normal average speed */
				const bool airspeed_valid = PX4_ISFINITE(_airspeed_sub.get().indicated_airspeed_m_s)
							    && (_airspeed_sub.get().indicated_airspeed_m_s > 0.0f)
							    && (hrt_elapsed_time(&_airspeed_sub.get().timestamp) < 1e6);

				if (!_parameters.airspeed_disabled && airspeed_valid) {
					/* prevent numerical drama by requiring 0.5 m/s minimal speed */
					airspeed = math::max(0.5f, _airspeed_sub.get().indicated_airspeed_m_s);

				} else {
					airspeed = _parameters.airspeed_trim;

					if (!airspeed_valid) {
						perf_count(_nonfinite_input_perf);
					}
				}

				/*
				 * For scaling our actuators using anything less than the min (close to stall)
				 * speed doesn't make any sense - its the strongest reasonable deflection we
				 * want to do in flight and its the baseline a human pilot would choose.
				 *
				 * Forcing the scaling to this value allows reasonable handheld tests.
				 */
				float airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min :
							 airspeed);

				/* Use min airspeed to calculate ground speed scaling region.
				 * Don't scale below gspd_scaling_trim
				 */
				float groundspeed = sqrtf(_global_pos.vel_n * _global_pos.vel_n +
							  _global_pos.vel_e * _global_pos.vel_e);
				float gspd_scaling_trim = (_parameters.airspeed_min * 0.6f);
				float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

				/* reset integrals where needed */
				if (_att_sp.roll_reset_integral) {
					_roll_ctrl.reset_integrator();
				}

				if (_att_sp.pitch_reset_integral) {
					_pitch_ctrl.reset_integrator();
				}

				if (_att_sp.yaw_reset_integral) {
					_yaw_ctrl.reset_integrator();
					_wheel_ctrl.reset_integrator();
				}

				/* Reset integrators if the aircraft is on ground
				 * or a multicopter (but not transitioning VTOL)
				 */
				if (_landed
				    || (_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode)) {

					_roll_ctrl.reset_integrator();
					_pitch_ctrl.reset_integrator();
					_yaw_ctrl.reset_integrator();
					_wheel_ctrl.reset_integrator();
				}

				float roll_sp = _att_sp.roll_body;
				float pitch_sp = _att_sp.pitch_body;
				float yaw_sp = _att_sp.yaw_body;

				/* Prepare data for attitude controllers */
				struct ECL_ControlData control_input = {};
				control_input.roll = euler_angles.phi();
				control_input.pitch = euler_angles.theta();
				control_input.yaw = euler_angles.psi();
				control_input.body_x_rate = _att.rollspeed;
				control_input.body_y_rate = _att.pitchspeed;
				control_input.body_z_rate = _att.yawspeed;
				control_input.roll_setpoint = roll_sp;
				control_input.pitch_setpoint = pitch_sp;
				control_input.yaw_setpoint = yaw_sp;
				control_input.airspeed_min = _parameters.airspeed_min;
				control_input.airspeed_max = _parameters.airspeed_max;
				control_input.airspeed = airspeed;
				control_input.scaler = airspeed_scaling;
				control_input.lock_integrator = lock_integrator;
				control_input.groundspeed = groundspeed;
				control_input.groundspeed_scaler = groundspeed_scaler;

				/* reset body angular rate limits on mode change */
				if ((_vcontrol_mode.flag_control_attitude_enabled != _flag_control_attitude_enabled_last) || params_updated) {
					if (_vcontrol_mode.flag_control_attitude_enabled) {
						_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));
						_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
						_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));
						_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

					} else {
						_roll_ctrl.set_max_rate(_parameters.acro_max_x_rate_rad);
						_pitch_ctrl.set_max_rate_pos(_parameters.acro_max_y_rate_rad);
						_pitch_ctrl.set_max_rate_neg(_parameters.acro_max_y_rate_rad);
						_yaw_ctrl.set_max_rate(_parameters.acro_max_z_rate_rad);
					}
				}

				_flag_control_attitude_enabled_last = _vcontrol_mode.flag_control_attitude_enabled;

				/* bi-linear interpolation over airspeed for actuator trim scheduling */
				float trim_roll = _parameters.trim_roll;
				float trim_pitch = _parameters.trim_pitch;
				float trim_yaw = _parameters.trim_yaw;

				if (airspeed < _parameters.airspeed_trim) {
					trim_roll += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_roll_vmin,
								   0.0f);
					trim_pitch += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_pitch_vmin,
								    0.0f);
					trim_yaw += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_yaw_vmin,
								  0.0f);

				} else {
					trim_roll += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
								   _parameters.dtrim_roll_vmax);
					trim_pitch += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
								    _parameters.dtrim_pitch_vmax);
					trim_yaw += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
								  _parameters.dtrim_yaw_vmax);
				}

				/* add trim increment if flaps are deployed  */
				trim_roll += _flaps_applied * _parameters.dtrim_roll_flaps;
				trim_pitch += _flaps_applied * _parameters.dtrim_pitch_flaps;

				/* Run attitude controllers */
				if (_vcontrol_mode.flag_control_attitude_enabled) {
					if (PX4_ISFINITE(roll_sp) && PX4_ISFINITE(pitch_sp)) {
						_roll_ctrl.control_attitude(control_input);
						_pitch_ctrl.control_attitude(control_input);
						_yaw_ctrl.control_attitude(control_input); //runs last, because is depending on output of roll and pitch attitude
						_wheel_ctrl.control_attitude(control_input);

						/* Update input data for rate controllers */
						control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
						control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
						control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

						/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
						float roll_u = _roll_ctrl.control_euler_rate(control_input);
						_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

						if (!PX4_ISFINITE(roll_u)) {
							_roll_ctrl.reset_integrator();
							perf_count(_nonfinite_output_perf);
						}

						float pitch_u = _pitch_ctrl.control_euler_rate(control_input);
						_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

						if (!PX4_ISFINITE(pitch_u)) {
							_pitch_ctrl.reset_integrator();
							perf_count(_nonfinite_output_perf);
						}

						float yaw_u = 0.0f;

						if (_parameters.w_en && _att_sp.fw_control_yaw) {
							yaw_u = _wheel_ctrl.control_bodyrate(control_input);

						} else {
							yaw_u = _yaw_ctrl.control_euler_rate(control_input);
						}

						_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

						/* add in manual rudder control in manual modes */
						if (_vcontrol_mode.flag_control_manual_enabled) {
							_actuators.control[actuator_controls_s::INDEX_YAW] += _manual.r;
						}

						if (!PX4_ISFINITE(yaw_u)) {
							_yaw_ctrl.reset_integrator();
							_wheel_ctrl.reset_integrator();
							perf_count(_nonfinite_output_perf);
						}

                        /////////////////////////////////////////////////////////////////////////////////////////////
                        ////////////////////===========>
                        /*Access to vertical_takeoff_controller() from Mission mode*/

                        /* Auto takeoff loop with GPS, in Mission Mode with Takeoff Waypoint*/
                        if (!_parameters.take_off_indoor) {
                            /* Trigger auto vertical takeoff when receive message _att_sp.decollage_custom from Pos Controller*/
                            if (_att_sp.decollage_custom && !take_off_trigger) {
                                mode_take_off_custom = true; // Start auto takeoff
                                present_time = hrt_absolute_time(); // timer pour les etapes du decollage
                                mode_seq = WAIT;
                                warnx("Trigger mode_take_off_custom from _att_sp.decollage_custom - Mission and Takeoff");

                                take_off_trigger = true;
                            }

                                /* Untrigger auto vertical takeoff when Pos Controller is done */
                            else if (!_att_sp.decollage_custom && take_off_trigger) {
                                // mode_take_off_custom = false; // Kill auto takeoff??
                                warnx("Untrigger mode_take_off_custom from _att_sp.decollage_custom - Mission and Takeoff");

                                take_off_trigger = false;
                            }
                        }

                        if (mode_take_off_custom) {
                            vertical_takeoff_controller();
                        }
                        ////<===========================////////////////////////////////////////////////////////////////
                        /////////////////////////////////////////////////////////////////////////////////////////////

                        /* If no custom takeoff, throttle comes from PositionController through _att_sp.thrust*/
                        else {
                            _actuators_airframe.control[1] = _parameters.take_off_prop_fly;
                            _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;

                            /* throttle passed through if it is finite and if no engine failure was detected */
                            _actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust)
                                                                                       &&
                                                                                       !_vehicle_status.engine_failure)
                                                                                      ? _att_sp.thrust : 0.0f;

                            /* scale effort by battery status */
                            if (_parameters.bat_scale_en &&
                                _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {

                                bool updated = false;
                                orb_check(_battery_status_sub, &updated);

                                if (updated) {
                                    battery_status_s battery_status = {};

                                    if (orb_copy(ORB_ID(battery_status), _battery_status_sub, &battery_status) ==
                                        PX4_OK) {
                                        if (battery_status.scale > 0.0f) {
                                            _battery_scale = battery_status.scale;
                                        }
                                    }
                                }

                                _actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_scale;
                            }
                        }

					} else {
						perf_count(_nonfinite_input_perf);
					}

					/*
					 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
					 * only once available
					 */
					_rates_sp.roll = _roll_ctrl.get_desired_bodyrate();
					_rates_sp.pitch = _pitch_ctrl.get_desired_bodyrate();
					_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

					_rates_sp.timestamp = hrt_absolute_time();

					if (_rate_sp_pub != nullptr) {
						/* publish the attitude rates setpoint */
						orb_publish(_rates_sp_id, _rate_sp_pub, &_rates_sp);

					} else if (_rates_sp_id) {
						/* advertise the attitude rates setpoint */
						_rate_sp_pub = orb_advertise(_rates_sp_id, &_rates_sp);
					}

				} else {
					// pure rate control
					_roll_ctrl.set_bodyrate_setpoint(_rates_sp.roll);
					_pitch_ctrl.set_bodyrate_setpoint(_rates_sp.pitch);
					_yaw_ctrl.set_bodyrate_setpoint(_rates_sp.yaw);

					float roll_u = _roll_ctrl.control_bodyrate(control_input);
					_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

					float pitch_u = _pitch_ctrl.control_bodyrate(control_input);
					_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

					float yaw_u = _yaw_ctrl.control_bodyrate(control_input);
					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_rates_sp.thrust) ? _rates_sp.thrust : 0.0f;
                    _actuators_airframe.control[1] = _parameters.take_off_prop_fly;
                    _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;
                }

				rate_ctrl_status_s rate_ctrl_status;
				rate_ctrl_status.timestamp = hrt_absolute_time();
				rate_ctrl_status.rollspeed = _att.rollspeed;
				rate_ctrl_status.pitchspeed = _att.pitchspeed;
				rate_ctrl_status.yawspeed = _att.yawspeed;
				rate_ctrl_status.rollspeed_integ = _roll_ctrl.get_integrator();
				rate_ctrl_status.pitchspeed_integ = _pitch_ctrl.get_integrator();
				rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();
				rate_ctrl_status.additional_integ1 = _wheel_ctrl.get_integrator();

				int instance;
				orb_publish_auto(ORB_ID(rate_ctrl_status), &_rate_ctrl_status_pub, &rate_ctrl_status, &instance, ORB_PRIO_DEFAULT);
			}


            /////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////===========> Etienne - Takeoff trigger when Indoor flight, No GPS

            /* Allow auto takeoff indoor or without GPS from _parameters.take_off_indoor*/
            if (_parameters.take_off_indoor) {
                /* Trigger auto vertical takeoff when in no GPS, manual/stab mode*/
                if (_vcontrol_mode.flag_armed && !take_off_trigger) {
                    mode_take_off_custom = true; // Start auto takeoff
                    present_time = hrt_absolute_time(); // timer pour les etapes du decollage
                    mode_seq = WAIT;
                    warnx("Trigger mode_take_off_custom from _parameters.take_off_indoor - Armed");

                    take_off_trigger = true;
                }

                    /* Untrigger auto vertical takeoff when in no GPS, manual/stab mode */
                else if (!_vcontrol_mode.flag_armed && take_off_trigger) {
                    mode_take_off_custom = false; // Kill auto takeoff
                    warnx("Untrigger mode_take_off_custom from _parameters.take_off_indoor - Unarmed");

                    take_off_trigger = false;
                }

                if (mode_take_off_custom) {
                    vertical_takeoff_controller();
                }
            }

            ///<===========================////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////

			// Add feed-forward from roll control output to yaw control output
			// This can be used to counteract the adverse yaw effect when rolling the plane
			_actuators.control[actuator_controls_s::INDEX_YAW] += _parameters.roll_to_yaw_ff * math::constrain(
						_actuators.control[actuator_controls_s::INDEX_ROLL], -1.0f, 1.0f);

			_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
			_actuators.control[5] = _manual.aux1;
			_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
			// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _att.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _att.timestamp;

			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_rates_enabled ||
			    _vcontrol_mode.flag_control_attitude_enabled ||
			    _vcontrol_mode.flag_control_manual_enabled) {
				/* publish the actuator controls */
				if (_actuators_0_pub != nullptr) {
					orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

				} else if (_actuators_id) {
					_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
				}

				if (_actuators_2_pub != nullptr) {
					/* publish the actuator controls*/
					orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_airframe);

				} else {
					/* advertise and publish */
					_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_airframe);
				}
			}
		}

		perf_end(_loop_perf);
	}
}

void FixedwingAttitudeControl::control_flaps(const float dt)
{
	/* default flaps to center */
	float flap_control = 0.0f;

	/* map flaps by default to manual if valid */
	if (PX4_ISFINITE(_manual.flaps) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_parameters.flaps_scale) > 0.01f) {
		flap_control = 0.5f * (_manual.flaps + 1.0f) * _parameters.flaps_scale;

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_parameters.flaps_scale) > 0.01f) {
		switch (_att_sp.apply_flaps) {
		case vehicle_attitude_setpoint_s::FLAPS_OFF : flap_control = 0.0f; // no flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_LAND : flap_control = 1.0f * _parameters.flaps_scale; // landing flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_TAKEOFF : flap_control = 1.0f * _parameters.flaps_scale *
					_parameters.flaps_takeoff_scale; // take-off flaps
			break;
		}
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaps_applied - flap_control) > 0.01f) {
		_flaps_applied += (_flaps_applied - flap_control) < 0 ? dt : -dt;

	} else {
		_flaps_applied = flap_control;
	}

	/* default flaperon to center */
	float flaperon_control = 0.0f;

	/* map flaperons by default to manual if valid */
	if (PX4_ISFINITE(_manual.aux2) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_parameters.flaperon_scale) > 0.01f) {
		flaperon_control = 0.5f * (_manual.aux2 + 1.0f) * _parameters.flaperon_scale;

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_parameters.flaperon_scale) > 0.01f) {
		flaperon_control = (_att_sp.apply_flaps == vehicle_attitude_setpoint_s::FLAPS_LAND) ? 1.0f *
				   _parameters.flaperon_scale : 0.0f;
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaperons_applied - flaperon_control) > 0.01f) {
		_flaperons_applied += (_flaperons_applied - flaperon_control) < 0 ? dt : -dt;

	} else {
		_flaperons_applied = flaperon_control;
	}
}

FixedwingAttitudeControl *FixedwingAttitudeControl::instantiate(int argc, char *argv[])
{
	return new FixedwingAttitudeControl();
}

int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("fw_att_controol",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ATTITUDE_CONTROL,
				      1500,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FixedwingAttitudeControl::print_status()
{
	PX4_INFO("Running");

	// perf?

	return 0;
}

int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}
