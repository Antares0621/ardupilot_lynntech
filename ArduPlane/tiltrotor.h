/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include "transition.h"
#include <unistd.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Relay/AP_Relay.h>
#include "AP_HAL/AP_HAL.h"


/*This value represents 90 degrees and is the true
  representation of servo max travel as opposed to SERVO_MAX
  which is used throughout the codebase*/
#define REAL_SERVO_MAX 9000.0   

class QuadPlane;
class AP_MotorsMulticopter;
class Tiltrotor_Transition;
class Tiltrotor
{
friend class QuadPlane;
friend class Plane;
friend class Tiltrotor_Transition;
public:

    Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors);

    bool enabled() const { return (enable > 0) && setup_complete;}

    void setup();

    void slew(float tilt);
    void binary_slew(bool forward);
    void update();
    void continuous_update();
    void binary_update();
    void vectoring();
    void update_transition_tracker(); // modified
    void mySleep(int milliseconds); // modified
    float get_tilt_percent_error(float target_val, float current_val); // modified
    float map_real_servo_angle_range_to_servomax_angle_range(float desired_angle_cd); // modified
    float map_servomax_angle_range_to_real_servo_angle_range(float desired_angle_cd); // modified
    float map_servomax_angle_range_to_bicopter_normalised_angle_range(float desired_angle_cd); // modified
    float map_bicopter_normalised_angle_range_to_servomax_angle_range(float desired_tilt); // modified
    float map_bicopter_normalised_angle_range_to_codebase_normalised_angle_range(float desired_value); // modified
    float map_normalised_pwm_input_to_forward_flight_tilt_angle_range(float desired_tilt); // modified
    float map_arspd_input_to_vtol_tilt_angle_range(); // modified
    void update_receive(); // modified
    void request_datastream(); //modified
    void bicopter_output(); 
    void tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul);
    void tilt_compensate(float *thrust, uint8_t num_motors);
    bool tilt_over_max_angle(void) const;

    bool is_motor_tilting(uint8_t motor) const {
        return tilt_mask.get() & (1U<<motor);
    }

    bool fully_fwd() const;
    bool fully_up() const;
    float tilt_max_change(bool up, bool in_flap_range = false) const;
    float get_fully_forward_tilt() const;
    float get_forward_flight_tilt() const;

    // update yaw target for tiltrotor transition
    void update_yaw_target();

    bool is_vectored() const { return enabled() && _is_vectored; }

    bool has_fw_motor() const { return _have_fw_motor; }

    bool has_vtol_motor() const { return _have_vtol_motor; }

    bool motors_active() const { return enabled() && _motors_active; }

    // true if the tilts have completed slewing
    // always return true if not enabled or not a continuous type
    bool tilt_angle_achieved() const { return !enabled() || (type != TILT_TYPE_CONTINUOUS) || angle_achieved; }

    AP_Int8 enable;
    AP_Int16 tilt_mask;
    AP_Int16 max_rate_up_dps;
    AP_Int16 max_rate_down_dps;
    AP_Int8  max_angle_deg;
    AP_Int8  type;
    AP_Float tilt_yaw_angle;
    AP_Float fixed_angle;
    AP_Float fixed_gain;
    AP_Float flap_angle_deg;
    AP_Float vtol_angle_deg; // modified
    AP_Float forward_flight_angle_deg; // modified
    AP_Float perc_error_tolerated_for_tilt; // modified
    AP_Float forward_flight_tilt_range_deg; // modified
    AP_Float forward_flight_tilt_rate_dps; // modified
    AP_Float arspd_noise_threshold_mps; // modified

    int transition_start_time; // modified
    int counter_for_transition_period_loop=0; // modified
    int transition_tracker=0; //modified
    float forward_flight_tilt=10000; // modified
    bool is_transition_done=false; // modified
    float transition_perc_err=10000; // modified
    float servo_tilt_angle_range; // max angle that servos can tilt from trim // modified
    float codebase_current_tilt=10000; // current tilt normalised between -1 and 1 // modified
    int vtol_mode_tilt_tracker=0; // modified
    float vtol_mode_tilt_start_time; //modified
    float vtol_flight_tilt; //modified
    float actual_forward_flight_angle_achieved_cd; // actual forward flight angle achieved after transition // modified
    float ff_tilt_upper_bound_cd; // forward flight upper tilt limit // modified
    float ff_tilt_lower_bound_cd; // forward flight lower tilt limit // modified
    float current_tilt;
    float current_throttle;
    bool _motors_active:1;
    float transition_yaw_cd;
    uint32_t transition_yaw_set_ms;
    bool _is_vectored;

    // types of tilt mechanisms
    enum {TILT_TYPE_CONTINUOUS    =0,
          TILT_TYPE_BINARY        =1,
          TILT_TYPE_VECTORED_YAW  =2,
          TILT_TYPE_BICOPTER      =3
    };

    static const struct AP_Param::GroupInfo var_info[];

private:

    bool setup_complete;

    // true if a fixed forward motor is setup
    bool _have_fw_motor;

    // true if all motors tilt with no fixed VTOL motor
    bool _have_vtol_motor;

    // true if the current tilt angle is equal to the desired
    // with slow tilt rates the tilt angle can lag
    bool angle_achieved;

    // refences for convenience
    QuadPlane& quadplane;
    AP_MotorsMulticopter*& motors;

    Tiltrotor_Transition* transition;

};

// Transition for separate left thrust quadplanes
class Tiltrotor_Transition : public SLT_Transition
{
friend class Tiltrotor;
public:

    Tiltrotor_Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tiltrotor& _tiltrotor):SLT_Transition(_quadplane, _motors), tiltrotor(_tiltrotor) {};

    bool update_yaw_target(float& yaw_target_cd) override;

    bool show_vtol_view() const override;

private:

    Tiltrotor& tiltrotor;

};
