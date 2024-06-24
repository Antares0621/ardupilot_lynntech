#include "tiltrotor.h"
#include "Plane.h"
#include <cstdlib>
#include <SRV_Channel/SRV_Channel.h>
#include "include/mavlink/v2.0/common/mavlink.h"
// #include "AP_HAL/AP_HAL.h"

// AP_HAL::UARTDriver* telem2_uart = hal.serial(2);


#if HAL_QUADPLANE_ENABLED
const AP_Param::GroupInfo Tiltrotor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Tiltrotor functionality
    // @Values: 0:Disable, 1:Enable
    // @Description: This enables Tiltrotor functionality
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
    AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0),

    // @Param: RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_UP", 3, Tiltrotor, max_rate_up_dps, 40),

    // @Param: MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("MAX", 4, Tiltrotor, max_angle_deg, 45),

    // @Param: TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover, Bicopter tiltrotor must use the tailsitter frame class (10)
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw,3:Bicopter
    AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS),

    // @Param: RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_DN", 6, Tiltrotor, max_rate_down_dps, 0),

    // @Param: YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output (fully back). This needs to be set in addition to Q_TILT_TYPE=2, to enable vectored control for yaw in tilt quadplanes. This is also used to limit the forward travel of bicopter tilts(Q_TILT_TYPE=3) when in VTOL modes.
    // @Range: 0 30
    AP_GROUPINFO("YAW_ANGLE", 7, Tiltrotor, tilt_yaw_angle, 0),

    // @Param: FIX_ANGLE
    // @DisplayName: Fixed wing tiltrotor angle
    // @Description: This is the angle the motors tilt down when at maximum output for forward flight. Set this to a non-zero value to enable vectoring for roll/pitch in forward flight on tilt-vectored aircraft
    // @Units: deg
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("FIX_ANGLE", 8, Tiltrotor, fixed_angle, 0),

    // @Param: FIX_GAIN
    // @DisplayName: Fixed wing tiltrotor gain
    // @Description: This is the gain for use of tilting motors in fixed wing flight for tilt vectored quadplanes
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FIX_GAIN", 9, Tiltrotor, fixed_gain, 0),

    // @Param: WING_FLAP
    // @DisplayName: Tiltrotor tilt angle that will be used as flap
    // @Description: For use on tilt wings, the wing will tilt up to this angle for flap, transition will be complete when the wing reaches this angle from the forward fight position, 0 disables
    // @Units: deg
    // @Increment: 1
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("WING_FLAP", 10, Tiltrotor, flap_angle_deg, 0),

    // @Param: VTOL_ANG
    // @DisplayName: Tiltrotor tilt angle that will represent target tilt angle for vtol (hover) mode 
    // @Description: currently only active for bicopter mode, the motors will tilt to this angle when in VTOL mode, transition to vtol will be complete when the motors reach this angle.
    // @Units: deg
    // @Increment: 1
    // @Range: -90 90
    // @User: Standard
    AP_GROUPINFO("VTOL_ANG", 11, Tiltrotor, vtol_angle_deg, -90),

    // @Param: FW_ANG
    // @DisplayName: Tiltrotor tilt angle that will represent target tilt angle for Fixed Wing (forward flight) Mode
    // @Description: currently only active for bicopter mode, the motors will tilt to this angle when in Fixed Wing mode, transition to Fixed Wing Mode will be complete when the motors reach this angle from the VTOL mode orientation.
    // @Units: deg
    // @Increment: 1
    // @Range: -90 90
    // @User: Standard
    AP_GROUPINFO("FW_ANG", 12, Tiltrotor, forward_flight_angle_deg, 0),

    // @Param: PERC_ERR
    // @DisplayName: percennt error tolerated when tilting to a certain angle e.g., transitioning to VTOL mode or transitioning from VTOL to FIXED WIND MODE
    // @Description: currently only active for bicopter mode, tightening this tolerance i.e. decreasing the number will allow more accurate tilting. since the tilt, a normalised representation of the angle is a float, setting the percent error to zero is not a good idea.
    // @Increment: 1
    // @Range: 0 0.5
    // @User: Standard
    AP_GROUPINFO("PERC_ERR", 13, Tiltrotor, perc_error_tolerated_for_tilt, 0.5),

    // @Param: FF_RANGE
    // @DisplayName: amount of motor tilt allowed in fixed wing mode
    // @Description: currently only active for bicopter mode, this is the amount of motor tilt available in fixed wing mode i.e. forward flight mode
    // @Increment: 1
    // @Range: 0 90
    // @User: Standard
    AP_GROUPINFO("FF_RANGE", 14, Tiltrotor, forward_flight_tilt_range_deg, 0),

    // @Param: FF_RATE
    // @DisplayName: motor tilt rate in fixed wing mode
    // @Description: currently only active for bicopter mode,this is the maximum speed at which the motor angle will change for a tiltrotor once in forward flight (FIXED WING MODE)
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("FF_RATE", 15, Tiltrotor, forward_flight_tilt_rate_dps, 40),

    // @Param: ARSPD_NT
    // @DisplayName: airspeed below which motor tilt remains at vtol specified angle, VTOL_ANG
    // @Description: currently only active for bicopter mode, airspeed below which tilt remains at vtol specified angle, VTOL_ANG
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ARSPD_NT", 16, Tiltrotor, arspd_noise_threshold_mps, 2),

    AP_GROUPEND
};

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */

Tiltrotor::Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Tiltrotor::setup()
{

    if (!enable.configured() && ((tilt_mask != 0) || (type == TILT_TYPE_BICOPTER))) {
        enable.set_and_save(1);
    }

    if (enable <= 0) {
        return;
    }

    _is_vectored = tilt_mask != 0 && type == TILT_TYPE_VECTORED_YAW;

    // true if a fixed forward motor is configured, either throttle, throttle left  or throttle right.
    // bicopter tiltrotors use throttle left and right as tilting motors, so they don't count in that case.
    _have_fw_motor = SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
                    ((SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) || SRV_Channels::function_assigned(SRV_Channel::k_throttleRight))
                        && (type != TILT_TYPE_BICOPTER));


    // check if there are any permanent VTOL motors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (motors->is_motor_enabled(i) && ((tilt_mask & (1U<<1)) == 0)) {
            // enabled motor not set in tilt mask
            _have_vtol_motor = true;
            break;
        }
    }

    if (_is_vectored) {
        // we will be using vectoring for yaw
        motors->disable_yaw_torque();
    }

    if (tilt_mask != 0) {
        // setup tilt compensation
        motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t));
        if (type == TILT_TYPE_VECTORED_YAW) {
            // setup tilt servos for vectored yaw
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRear,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearLeft, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearRight, 1000);
        }
    }

    transition = new Tiltrotor_Transition(quadplane, motors, *this);
    if (!transition) {
        AP_BoardConfig::allocation_error("tiltrotor transition");
    }
    quadplane.transition = transition;

    setup_complete = true;
}

/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
 */
float Tiltrotor::tilt_max_change(bool up, bool in_flap_range) const
{
    // Initialize tilt rate
    float rate = forward_flight_tilt_rate_dps;

    // Determine tilt rate
    // if in transition from forward flight to vtol mode
    if (plane.arming.is_armed_and_safety_off() && (transition_tracker == 1)){
        if (up || max_rate_down_dps <= 0) {
            rate = max_rate_up_dps;
        } else {
            rate = max_rate_down_dps;
        }
    // if in vtol mode or forward flight mode (i.e., post transition),
    } else if((plane.arming.is_armed_and_safety_off() && transition_tracker == 0) || (transition_tracker > 1)) {
        rate = forward_flight_tilt_rate_dps;
    }

    if (type != TILT_TYPE_BINARY && !up && !in_flap_range) {
        bool fast_tilt = false;
        if (plane.control_mode == &plane.mode_manual) {
            fast_tilt = true;
        }
        if (plane.arming.is_armed_and_safety_off() && !quadplane.in_vtol_mode() && !quadplane.assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            rate = MAX(rate, 90);
        }
    }
    return rate * plane.G_Dt * (1/90.0);
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
void Tiltrotor::slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<current_tilt, newtilt > get_fully_forward_tilt());
    current_tilt = constrain_float(newtilt, current_tilt-max_change, current_tilt+max_change);

    angle_achieved = is_equal(newtilt, current_tilt);
    
    /*linear mapping (interpolation) of bicopter servo range to general code servo range.
    This ensures that in bicopter mode, VTOL mode motor tilt is at min pwm 
    and FIXED WING motor tilt is at trim pwm*/
    if (type == TILT_TYPE_BICOPTER){
        // gcs().send_text(MAV_SEVERITY_INFO,"SLEw-0000-WWWWWW-AAAAAAAA-AAAA-000000000000");
        codebase_current_tilt = map_bicopter_normalised_angle_range_to_codebase_normalised_angle_range(current_tilt);
        // gcs().send_text(MAV_SEVERITY_INFO,"current tilt, codebase_current_tilt: %.4f, %.4f", (float)current_tilt, float(codebase_current_tilt));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  SERVO_MAX * codebase_current_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, SERVO_MAX * codebase_current_tilt);
    }

    // translate to 0..1000 range and output
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
    
}

// return the current tilt value that represents forward flight
// tilt wings can sustain forward flight with some amount of wing tilt
float Tiltrotor::get_fully_forward_tilt() const
{
    return 1.0 - (flap_angle_deg * (1/90.0));
}

// return the target tilt value for forward flight
float Tiltrotor::get_forward_flight_tilt() const
{
    return 1.0 - ((flap_angle_deg * (1/90.0)) * SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto) * 0.01);
}

/*
  update motor tilt for continuous tilt servos
 */
void Tiltrotor::continuous_update(void)
{
    // default to inactive
    _motors_active = false;

    // the maximum rate of throttle change
    float max_change;

    if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor

        // option set then if disarmed move to VTOL position to prevent ground strikes, allow tilt forward in manual mode for testing
        const bool disarmed_tilt_up = !plane.arming.is_armed_and_safety_off() && (plane.control_mode != &plane.mode_manual) && quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT_UP);
        slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt());

        max_change = tilt_max_change(false);

        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
        if (current_tilt < get_fully_forward_tilt()) {
            current_throttle = constrain_float(new_throttle,
                                                    current_throttle-max_change,
                                                    current_throttle+max_change);
        } else {
            current_throttle = new_throttle;
        }
        if (!plane.arming.is_armed_and_safety_off()) {
            current_throttle = 0;
        } else {
            // prevent motor shutdown
            _motors_active = true;
        }
        if (!quadplane.motor_test.running) {
            // the motors are all the way forward, start using them for fwd thrust
            const uint16_t mask = is_zero(current_throttle)?0U:tilt_mask.get();
            motors->output_motor_mask(current_throttle, mask, plane.rudder_dt);
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<current_throttle);
    current_throttle = constrain_float(motors_throttle,
                                            current_throttle-max_change,
                                            current_throttle+max_change);

    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 5 strategies we will use:

      1) With use of a forward throttle controlled by Q_FWD_THR_GAIN in
         VTOL modes except Q_AUTOTUNE determined by Q_FWD_THR_USE. We set the angle based on a calculated
         forward throttle.

      2) With manual forward throttle control we set the angle based on the
         RC input demanded forward throttle for QACRO, QSTABILIZE and QHOVER.

      3) Without a RC input or calculated forward throttle value, the angle
         will be set to zero in QAUTOTUNE, QACRO, QSTABILIZE and QHOVER.
         This enables these modes to be used as a safe recovery mode.

      4) In fixed wing assisted flight or velocity controlled modes we will
         set the angle based on the demanded forward throttle, with a maximum
         tilt given by Q_TILT_MAX. This relies on Q_FWD_THR_GAIN or Q_VFWD_GAIN
         being set.

      5) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */

#if QAUTOTUNE_ENABLED
    if (plane.control_mode == &plane.mode_qautotune) {
        slew(0);
        return;
    }
#endif

    if (!quadplane.assisted_flight &&
        quadplane.get_vfwd_method() == QuadPlane::ActiveFwdThr::NEW &&
        quadplane.is_flying_vtol())
    {
        // We are using the rotor tilt functionality controlled by Q_FWD_THR_GAIN which can
        // operate in all VTOL modes except Q_AUTOTUNE. Forward rotor tilt is used to produce
        // forward thrust equivalent to what would have been produced by a forward thrust motor
        // set to quadplane.forward_throttle_pct()
        const float fwd_g_demand = 0.01f * quadplane.forward_throttle_pct() / plane.quadplane.q_fwd_thr_gain;
        const float fwd_tilt_deg = MIN(degrees(atanf(fwd_g_demand)), (float)max_angle_deg);
        slew(MIN(fwd_tilt_deg * (1/90.0), get_forward_flight_tilt()));
        return;
    } else if (!quadplane.assisted_flight &&
               (plane.control_mode == &plane.mode_qacro ||
               plane.control_mode == &plane.mode_qstabilize ||
               plane.control_mode == &plane.mode_qhover))
    {
        if (quadplane.rc_fwd_thr_ch == nullptr) {
            // no manual throttle control, set angle to zero
            slew(0);
        } else {
            // manual control of forward throttle up to max VTOL angle
            float settilt = .01f * quadplane.forward_throttle_pct();
            slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
        }
        return;
    }

    if (quadplane.assisted_flight &&
        transition->transition_state >= Tiltrotor_Transition::TRANSITION_TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        slew(get_forward_flight_tilt());
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
       float settilt = constrain_float((SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)-MAX(plane.aparm.throttle_min.get(),0)) * 0.02, 0, 1);
       slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
 */
void Tiltrotor::binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    float max_change = tilt_max_change(!forward);
    if (forward) {
        current_tilt = constrain_float(current_tilt+max_change, 0, 1);
    } else {
        current_tilt = constrain_float(current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
 */
void Tiltrotor::binary_update(void)
{
    // // motors always active
    // _motors_active = true;

    // if (!quadplane.in_vtol_mode()) {
    //     // we are in pure fixed wing mode. Move the tiltable motors
    //     // all the way forward and run them as a forward motor
    //     binary_slew(true);

    //     float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
    //     if (current_tilt >= 1) {
    //         const uint16_t mask = is_zero(new_throttle)?0U:tilt_mask.get();
    //         // the motors are all the way forward, start using them for fwd thrust
    //         motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
    //     }
    // } else {
    //     binary_slew(false);
    // }
    return;
}


/*
  update motor tilt
 */
void Tiltrotor::update(void)
{
    if (!enabled() || tilt_mask == 0) {
        // no motors to tilt
        return;
    }

    if (type == TILT_TYPE_BINARY) {
        binary_update();
    } else {
        continuous_update();
    }

    if (type == TILT_TYPE_VECTORED_YAW) {
        vectoring();
    }
}

/*
  tilt compensation for angle of tilt. When the rotors are tilted the
  roll effect of differential thrust on the tilted rotors is decreased
  and the yaw effect increased
  We have two factors we apply.

  1) when we are transitioning to fwd flight we scale the tilted rotors by 1/cos(angle). This pushes us towards more flight speed

  2) when we are transitioning to hover we scale the non-tilted rotors by cos(angle). This pushes us towards lower fwd thrust

  We also apply an equalisation to the tilted motors in proportion to
  how much tilt we have. This smoothly reduces the impact of the roll
  gains as we tilt further forward.

  For yaw, we apply differential thrust in proportion to the demanded
  yaw control and sin of the tilt angle

  Finally we ensure no requested thrust is over 1 by scaling back all
  motors so the largest thrust is at most 1.0
 */
void Tiltrotor::tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul)
{
    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factors first
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= non_tilted_mul;
        } else {
            thrust[i] *= tilted_mul;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;
    const float sin_tilt = sinf(radians(current_tilt*90));
    // yaw_gain relates the amount of differential thrust we get from
    // tilt, so that the scaling of the yaw control is the same at any
    // tilt angle
    const float yaw_gain = sinf(radians(tilt_yaw_angle));
    const float avg_tilt_thrust = tilt_total / tilt_count;

    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            // as we tilt we need to reduce the impact of the roll
            // controller. This simple method keeps the same average,
            // but moves us to no roll control as the angle increases
            thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1-current_tilt);
            // add in differential thrust for yaw control, scaled by tilt angle
            const float diff_thrust = motors->get_roll_factor(i) * (motors->get_yaw()+motors->get_yaw_ff()) * sin_tilt * yaw_gain;
            thrust[i] += diff_thrust;
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the motors then reduce all motors
    // to keep them in proportion to the original thrust. This helps
    // maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void Tiltrotor::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }
    if (quadplane.in_vtol_mode()) {
        // we are transitioning to VTOL flight
        const float tilt_factor = cosf(radians(current_tilt*90));
        tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
    } else {
        float inv_tilt_factor;
        if (current_tilt > 0.98f) {
            inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
        } else {
            inv_tilt_factor = 1.0 / cosf(radians(current_tilt*90));
        }
        tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
    }
}

/*
  return true if the rotors are fully tilted forward
 */
bool Tiltrotor::fully_fwd(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt >= get_fully_forward_tilt());
}

/*
  return true if the rotors are fully tilted up
 */
bool Tiltrotor::fully_up(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt <= 0);
}

/*
  control vectoring for tilt multicopters
 */
void Tiltrotor::vectoring(void)
{
//     // total angle the tilt can go through
//     const float total_angle = 90 + tilt_yaw_angle + fixed_angle;
//     // output value (0 to 1) to get motors pointed straight up
//     const float zero_out = tilt_yaw_angle / total_angle;
//     const float fixed_tilt_limit = fixed_angle / total_angle;
//     const float level_out = 1.0 - fixed_tilt_limit;

//     // calculate the basic tilt amount from current_tilt
//     float base_output = zero_out + (current_tilt * (level_out - zero_out));
//     // for testing when disarmed, apply vectored yaw in proportion to rudder stick
//     // Wait TILT_DELAY_MS after disarming to allow props to spin down first.
//     constexpr uint32_t TILT_DELAY_MS = 3000;
//     uint32_t now = AP_HAL::millis();
//     if (!plane.arming.is_armed_and_safety_off() && plane.quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT)) {
//         // this test is subject to wrapping at ~49 days, but the consequences are insignificant
//         if ((now - hal.util->get_last_armed_change()) > TILT_DELAY_MS) {
//             if (quadplane.in_vtol_mode()) {
//                 float yaw_out = plane.channel_rudder->get_control_in();
//                 yaw_out /= plane.channel_rudder->get_range();
//                 float yaw_range = zero_out;

//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output,0,1));
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
//             } else {
//                 // fixed wing tilt
//                 const float gain = fixed_gain * fixed_tilt_limit;
//                 // base the tilt on elevon mixing, which means it
//                 // takes account of the MIXING_GAIN. The rear tilt is
//                 // based on elevator
//                 const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
//                 const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
//                 const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
//                 // front tilt is effective canards, so need to swap and use negative. Rear motors are treated live elevons.
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
//                 SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
//             }
//         }
//         return;
//     }

//     const bool no_yaw = tilt_over_max_angle();
//     if (no_yaw) {
//         // fixed wing  We need to apply inverse scaling with throttle, and remove the surface speed scaling as
//         // we don't want tilt impacted by airspeed
//         const float scaler = plane.control_mode == &plane.mode_manual?1:(quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
//         const float gain = fixed_gain * fixed_tilt_limit * scaler;
//         const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
//         const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
//         const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
//     } else {
//         const float yaw_out = motors->get_yaw()+motors->get_yaw_ff();
//         const float roll_out = motors->get_roll()+motors->get_roll_ff();
//         const float yaw_range = zero_out;

//         // Scaling yaw with throttle
//         const float throttle = motors->get_throttle_out();
//         const float scale_min = 0.5;
//         const float scale_max = 2.0;
//         float throttle_scaler = scale_max;
//         if (is_positive(throttle)) {
//             throttle_scaler = constrain_float(motors->get_throttle_hover() / throttle, scale_min, scale_max);
//         }

//         // now apply vectored thrust for yaw and roll.
//         const float tilt_rad = radians(current_tilt*90);
//         const float sin_tilt = sinf(tilt_rad);
//         const float cos_tilt = cosf(tilt_rad);
//         // the MotorsMatrix library normalises roll factor to 0.5, so
//         // we need to use the same factor here to keep the same roll
//         // gains when tilted as we have when not tilted
//         const float avg_roll_factor = 0.5;
//         float tilt_scale = throttle_scaler * yaw_out * cos_tilt + avg_roll_factor * roll_out * sin_tilt;

//         if (fabsf(tilt_scale) > 1.0) {
//             tilt_scale = constrain_float(tilt_scale, -1.0, 1.0);
//             motors->limit.yaw = true;
//         }

//         const float tilt_offset = tilt_scale * yaw_range;

//         float left_tilt = base_output + tilt_offset;
//         float right_tilt = base_output - tilt_offset;

//         // if output saturation of both left and right then set yaw limit flag
//         if (((left_tilt > 1.0) || (left_tilt < 0.0)) &&
//             ((right_tilt > 1.0) || (right_tilt < 0.0))) {
//             motors->limit.yaw = true;
//         }

//         // constrain and scale to ouput range
//         left_tilt = constrain_float(left_tilt,0.0,1.0) * 1000.0;
//         right_tilt = constrain_float(right_tilt,0.0,1.0) * 1000.0;

//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_tilt);
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_tilt);
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 1000.0 * constrain_float(base_output,0.0,1.0));
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft, left_tilt);
//         SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, right_tilt);
//     }
    return;
}



/*linear mapping (lin. interpolation) of real angle demanded in range -90 to 90 deg
to servo angular range between -SERVO_MAX and SERVO_MAX
 */
float Tiltrotor::map_real_servo_angle_range_to_servomax_angle_range(float desired_angle_cd){
    //desired angle is past in centidegrees
    // gcs().send_text(MAV_SEVERITY_INFO,"real_servo_angle_range_to_servomax: %.4f", (float)(SERVO_MAX/REAL_SERVO_MAX)*(desired_angle*100 + REAL_SERVO_MAX) - SERVO_MAX);
    return  (SERVO_MAX/REAL_SERVO_MAX)*(desired_angle_cd + REAL_SERVO_MAX) - SERVO_MAX;
}

/*linear mapping (lin. interpolation) of servo angular range between -SERVO_MAX and SERVO_MAX
to real angle demanded in range -90 to 90 deg
both input and output are in centidegrees
 */
float Tiltrotor::map_servomax_angle_range_to_real_servo_angle_range(float desired_angle_cd){
    return  (REAL_SERVO_MAX/SERVO_MAX)*(desired_angle_cd + SERVO_MAX) - REAL_SERVO_MAX;
}

/*linear mapping (lin. interpolation) from servo angle range 
between -SERVO_MAX  and SERVO_MAX to bicopter normalised angles
between 0 and 1
input is in centidegrees and output is a normalized value
 */
float Tiltrotor::map_servomax_angle_range_to_bicopter_normalised_angle_range(float desired_angle_cd){
    //
    // gcs().send_text(MAV_SEVERITY_INFO,"servomax_angle_range_to_bicopter: %.4f", (float)(desired_angle + SERVO_MAX)/(2*SERVO_MAX));
    return (desired_angle_cd + SERVO_MAX)/(2*SERVO_MAX);
}

/*linear mapping (lin. interpolation) from bicopter normalised angles
between 0 and 1 to servo angle range between -SERVO_MAX  and SERVO_MAX
input is a normalized value and output is an angle in centidegrees
 */
float Tiltrotor::map_bicopter_normalised_angle_range_to_servomax_angle_range(float desired_tilt){
    return (2*SERVO_MAX)*desired_tilt - SERVO_MAX;
}

/*linear mapping (lin. interpolation) from normalised_pwm_input
between -1 and 1 to forward_flight_tilt_angle range 
between ff_tilt_lower_bound_cd  and ff_tilt_upper_bound_cd
input is a normalized value and output is an angle in centidegrees
 */
float Tiltrotor::map_normalised_pwm_input_to_forward_flight_tilt_angle_range(float desired_tilt){
    // gcs().send_text(MAV_SEVERITY_INFO,"current tilt: %.1f, %.1f, %.1f, %.1f, %.4f, %.1f", (float)actual_forward_flight_angle_achieved_cd, (float)forward_flight_tilt_range_deg, (float)ff_tilt_lower_bound_cd, (float)ff_tilt_upper_bound_cd, (float)desired_tilt, (float)((0.5)*(ff_tilt_upper_bound_cd - ff_tilt_lower_bound_cd)*(desired_tilt + 1) + ff_tilt_lower_bound_cd));
    return (0.5)*(ff_tilt_upper_bound_cd - ff_tilt_lower_bound_cd)*(desired_tilt + 1) + ff_tilt_lower_bound_cd;
}

/*linear mapping (lin. interpolation) from arspd_input
between arspd_noise_threshold_mps and plane.aparm.airspeed_min (transition airspeed) to vtol tilt range 
between vtol_angle_deg and forward_flight_angle_deg both using
input is in m/s (mps) and output is an angle in centidegrees
 */
float Tiltrotor::map_arspd_input_to_vtol_tilt_angle_range(){
    float aspeed;
	quadplane.ahrs.airspeed_estimate(aspeed);
    // gcs().send_text(MAV_SEVERITY_INFO,"current tilt: %.1f, %.1f, %.1f, %.1f, %.4f, %.1f", (float)forward_flight_angle_deg, (float)vtol_angle_deg, (float)plane.aparm.airspeed_min, (float)arspd_noise_threshold_mps, (float)aspeed, (float)(((forward_flight_angle_deg - vtol_angle_deg)/(plane.aparm.airspeed_min - arspd_noise_threshold_mps))*(aspeed - arspd_noise_threshold_mps) + vtol_angle_deg));
    return ((forward_flight_angle_deg*100 - vtol_angle_deg*100)/(plane.aparm.airspeed_min - arspd_noise_threshold_mps))*(aspeed - arspd_noise_threshold_mps) + vtol_angle_deg*100;
}

/*linear mapping (lin. interpolation) from bicopter normalised angles 
between 0 and 1 to codebase normalised angles between -1 and 1
 */
float Tiltrotor::map_bicopter_normalised_angle_range_to_codebase_normalised_angle_range(float desired_value){
    return 2*desired_value - 1;
}

/* 
 function which will be executed when in fixed wing mode 
 to update the transition tracker
*/
void Tiltrotor::update_transition_tracker(){
	transition_tracker += 1;
}

/* calculate tilt percent error from desired tilt */
float Tiltrotor::get_tilt_percent_error(float target_val, float current_val){
    // handle a target value of zero (0)
    if (is_zero(target_val)){
        if (abs((target_val - current_val)*100) < perc_error_tolerated_for_tilt){
            return abs(target_val - current_val);
        } else {
            return abs((target_val - current_val)*100);
        }
    }
	return abs((target_val - current_val)/target_val)*100;
}


void Tiltrotor::request_datastream(){
  // Create a MAVLink message for requesting data stream
    mavlink_message_t requestMsg;
    mavlink_msg_request_data_stream_pack(1, 1, &requestMsg, 1, 158,
                                        MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1); // Request IMU data stream

    // Serialize and send the request message
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &requestMsg);
    hal.serial(2)->write(buf, len);

    // Print a message to the Serial Monitor for debugging
    // Serial.println("Requested Datastream from ArduPilot");
    // Serial.println(received_sysid);
}


// receive new mavlink packets from teensy
void Tiltrotor::update_receive()
{
    // // do absolutely nothing if we are locked
    // if (locked()) {
    //     return;
    // }

    mavlink_message_t msg;
    mavlink_status_t status;
    // uint32_t tstart_us = AP_HAL::micros();
    // uint32_t now_ms = AP_HAL::millis();

    status.packet_rx_drop_count = 0;

    if (hal.serial(2)->available() > 0){
        gcs().send_text(MAV_SEVERITY_INFO, "WE ARE INNNN JJ-1");
        const uint8_t c = (uint8_t)hal.serial(2)->read();

        // Try to get a new message
        if (mavlink_parse_char(2, c, &msg, &status)) {
            gcs().send_text(MAV_SEVERITY_INFO, "%d", (uint8_t) c);
            gcs().send_text(MAV_SEVERITY_INFO, "%d", (uint8_t) 2);
            gcs().send_text(MAV_SEVERITY_INFO, "WE ARE INNNN JJ");
            // hal.util->persistent_data.last_mavlink_msgid = msg.msgid;
        }
    }
}


/*
  control bicopter tiltrotors
 */
void Tiltrotor::bicopter_output(void){
    ///GPIO Relay for motor direction
    // AP_Relay*_relay = AP::relay();
    // gcs().send_text(MAV_SEVERITY_INFO, "%d", (int)_relay->enabled(0));
    
    // Receive and process MAVLink messages
    // request_datastream();
    // update_receive();

    if (transition_tracker == 0){
        /* If motors not already in  VTOL orientation,
        tilt motors to VTOL orientation*/
        

        /*this initialisation and update is useful for when the vehicle is armed.
        so that if we want to be able to tilt the motors in VTOL mode,
        we will be able to skip this code block which tries to maintain the motors at a constant orientation */
        if (vtol_mode_tilt_tracker==0){
            const uint32_t now = AP_HAL::millis();
            vtol_mode_tilt_start_time = now;
            // gcs().send_text(MAV_SEVERITY_INFO,"Start Time %.3f", (float)vtol_mode_tilt_start_time);
            vtol_mode_tilt_tracker += 1;
            // gcs().send_text(MAV_SEVERITY_INFO,"vtol_angle_deg: %.4f", (float)vtol_angle_deg);
            // using PWM corresponding to vtol_angle_deg for VTOL motor orientation
            vtol_flight_tilt = map_servomax_angle_range_to_bicopter_normalised_angle_range(map_real_servo_angle_range_to_servomax_angle_range(vtol_angle_deg*100));
            slew(vtol_flight_tilt);
        }

        /*keep tilting motors to get to VTOL orientation*/
        if (vtol_mode_tilt_tracker==1){
            // using PWM corresponding to vtol_angle_deg for VTOL motor orientation
            slew(vtol_flight_tilt);
            const uint32_t now = AP_HAL::millis();
            /*Give the system 10 seconds to start up
            before checking if VTOL mode motor orientation is accurate */
            if (((now - vtol_mode_tilt_start_time)/1000) > 10) {
                /* if in VTOL orientation, update vtol_mode_tilt_tracker
                   to avoid executing this code block after desired orientation is reached*/
                if (get_tilt_percent_error(vtol_flight_tilt, current_tilt) < perc_error_tolerated_for_tilt){
                  vtol_mode_tilt_tracker += 1;
                }
            }
        }
    }

	if (plane.arming.is_armed_and_safety_off()){
	
		/* default to active
		  prevent motor shutdown*/
		_motors_active = true;

		// the maximum rate of throttle change
		// float max_change;

		// get the airspeed estimate
		float aspeed;
		bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
		
		/* FIXED WING MODE ---------------------------------------------------------------------------------*/
		/* if we are armed and the safety is off,
		 if the arspd is greater than the minimum arspd to transition (arspd_fbw_min)
		 or we have already entered the fixed wing mode
		 then execute this loop which controls both the transition to fixed wing mode 
		 and what happens when in fixed wing mode*/
		if ((have_airspeed && (aspeed >= plane.aparm.airspeed_min)) || transition_tracker > 0){

			// update the transition tracker 
			update_transition_tracker();
            // gcs().send_text(MAV_SEVERITY_INFO,"Transition tracker %d", (int)transition_tracker);
			
			/* if transition_tracker is 1 after the update
			 then tilt motors to fully forward position*/
			if (transition_tracker == 1){
                gcs().send_text(MAV_SEVERITY_INFO,"FIXED WING MODE");

				// record the time the transition begins
                const uint32_t now = AP_HAL::millis();
				transition_start_time = now; // transition beginning time
                
                // tilt motors to fully forward angle
                // using PWM corresponding to forward_flight_angle_deg for VTOL motor orientation
                forward_flight_tilt = map_servomax_angle_range_to_bicopter_normalised_angle_range(map_real_servo_angle_range_to_servomax_angle_range(forward_flight_angle_deg*100));
				slew(forward_flight_tilt); 

                // error wrt current tilt and desired forward_flight_tilt
                transition_perc_err = get_tilt_percent_error(forward_flight_tilt, current_tilt);
			}

			//keep this block running till motors are fully forward and transition is complete.
			if ((transition_perc_err > perc_error_tolerated_for_tilt) && !is_transition_done){
                slew(forward_flight_tilt);
                transition_perc_err = get_tilt_percent_error(forward_flight_tilt, current_tilt);
                return;
            }

            // transition complete
            counter_for_transition_period_loop += 1;
            if (counter_for_transition_period_loop == 1){
                is_transition_done = true;
                actual_forward_flight_angle_achieved_cd = map_servomax_angle_range_to_real_servo_angle_range(map_bicopter_normalised_angle_range_to_servomax_angle_range(current_tilt));
                gcs().send_text(MAV_SEVERITY_INFO,"Final tilt for transition done: %.4f", (float)current_tilt);
                gcs().send_text(MAV_SEVERITY_INFO,"Angle for transition done, deg: %.4f", (float)actual_forward_flight_angle_achieved_cd*0.01);
                gcs().send_text(MAV_SEVERITY_INFO,"target forward_flight_tilt: %.4f", (float)forward_flight_tilt);
                gcs().send_text(MAV_SEVERITY_INFO,"Final tilt perc error: %.4f", (float)transition_perc_err);
                gcs().send_text(MAV_SEVERITY_INFO,"Final perc err condition met? %d", (int)(transition_perc_err <= perc_error_tolerated_for_tilt));
                gcs().send_text(MAV_SEVERITY_INFO,"transition_done? %d", (int)is_transition_done);
            }
            
			/*Execute the rest of code that
			  defines how the vehicle will be controlled 
			  when it is in fixed wing mode*/

            /*tilt the motors aboout the specified forward flight transition angle within a bounded range of tilt*/
            // set the bounds of the tilt
            ff_tilt_lower_bound_cd = - forward_flight_tilt_range_deg*100 + actual_forward_flight_angle_achieved_cd;
            ff_tilt_upper_bound_cd = forward_flight_tilt_range_deg*100 + actual_forward_flight_angle_achieved_cd;
            // we are doing passthrough from rc channel 2 input (i.e., the pitch input) to tilt output for the motortilting
            RC_Channel *chan2 = rc().channel(2-1);
            // gcs().send_text(MAV_SEVERITY_INFO,"chan 2 calc angle %.1f", (float)map_normalised_pwm_input_to_forward_flight_tilt_angle_range(c->norm_input_dz()));
            slew(map_servomax_angle_range_to_bicopter_normalised_angle_range(map_real_servo_angle_range_to_servomax_angle_range(map_normalised_pwm_input_to_forward_flight_tilt_angle_range(chan2->norm_input_dz()))));
			return;
		}
		/* VTOL MODE ---------------------------------------------------------------------------------*/
		/*Execute the rest of code that
		  defines how the vehicle will be controlled 
		  when it is in VTOL mode*/
          
        /* if the arspd is below some airspeed noise threshold, then keep the motors tilted at vtol angle
        else if the airspeed is between the airspeed noise threshold and the transition angle, tilt the motors as a 
        function of the airspeed*/
        if (have_airspeed && ((arspd_noise_threshold_mps < aspeed) && (aspeed < plane.aparm.airspeed_min))){
            slew(map_servomax_angle_range_to_bicopter_normalised_angle_range(map_real_servo_angle_range_to_servomax_angle_range(map_arspd_input_to_vtol_tilt_angle_range())));
        }
		return;
			
	} else {
	/* DISARMED VEHICLE---------------------------------------------------------------------------------*/
	    /* if the vehicle is disarmed, 
        then reset transition tracker variables*/
		transition_tracker = 0;
		is_transition_done = false;
        counter_for_transition_period_loop = 0;
      vtol_mode_tilt_tracker = 0;
	}
}


/*
  when doing a forward transition of a tilt-vectored quadplane we use
  euler angle control to maintain good yaw. This updates the yaw
  target based on pilot input and target roll
 */
void Tiltrotor::update_yaw_target(void)
{
    // uint32_t now = AP_HAL::millis();
    // if (now - transition_yaw_set_ms > 100 ||
    //     !is_zero(quadplane.get_pilot_input_yaw_rate_cds())) {
    //     // lock initial yaw when transition is started or when
    //     // pilot commands a yaw change. This allows us to track
    //     // straight in transitions for tilt-vectored planes, but
    //     // allows for turns when level transition is not wanted
    //     transition_yaw_cd = quadplane.ahrs.yaw_sensor;
    // }

    // /*
    //   now calculate the equivalent yaw rate for a coordinated turn for
    //   the desired bank angle given the airspeed
    //  */
    // float aspeed;
    // bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
    // if (have_airspeed && labs(plane.nav_roll_cd)>1000) {
    //     float dt = (now - transition_yaw_set_ms) * 0.001;
    //     // calculate the yaw rate to achieve the desired turn rate
    //     const float airspeed_min = MAX(plane.aparm.airspeed_min,5);
    //     const float yaw_rate_cds = fixedwing_turn_rate(plane.nav_roll_cd*0.01, MAX(aspeed,airspeed_min))*100;
    //     transition_yaw_cd += yaw_rate_cds * dt;
    // }
    // transition_yaw_set_ms = now;
}

bool Tiltrotor_Transition::update_yaw_target(float& yaw_target_cd)
{
    // if (!(tiltrotor.is_vectored() &&
    //     transition_state <= TRANSITION_TIMER)) {
    //     return false;
    // }
    // tiltrotor.update_yaw_target();
    // yaw_target_cd = tiltrotor.transition_yaw_cd;
    // return true;
    return false; // modified
}

// return true if we should show VTOL view
bool Tiltrotor_Transition::show_vtol_view() const
{
    // bool show_vtol = quadplane.in_vtol_mode();

    // if (!show_vtol && tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER) {
    //     // we use multirotor controls during fwd transition for
    //     // vectored yaw vehicles
    //     return true;
    // }

    // return show_vtol;
    return false;
}

// return true if we are tilted over the max angle threshold
bool Tiltrotor::tilt_over_max_angle(void) const
{
    // const float tilt_threshold = (max_angle_deg/90.0f);
    // return (current_tilt > MIN(tilt_threshold, get_forward_flight_tilt()));
    return false;
}

#endif  // HAL_QUADPLANE_ENABLED
