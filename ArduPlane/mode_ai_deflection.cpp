#include "mode.h"
#include "Plane.h"

ModeAI_Deflection::ModeAI_Deflection()
{
    
}

bool ModeAI_Deflection::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;
    
    
    _servoOutput.stickElevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    _servoOutput.stickRudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);

    _servoOutput.stickAileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    _servoOutput.stickThrottle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);


    return true;
}

void ModeAI_Deflection::_exit()
{
    if (plane.g.auto_trim > 0) {
        plane.trim_radio();
    }
}

void ModeAI_Deflection::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, _servoOutput.stickElevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, _servoOutput.stickRudder);

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, _servoOutput.stickAileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _servoOutput.stickThrottle);

    // plane.steering_control.steering = plane.steering_control.rudder = _servoOutput.stickRudder;

    // // Test to fix sim:
    // uint16_t PWMAileron, PWMElevator, PWMThrottle, PWMRudder;
    // SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, PWMAileron);
    // SRV_Channels::get_output_pwm(SRV_Channel::k_elevator, PWMElevator);
    // SRV_Channels::get_output_pwm(SRV_Channel::k_throttle, PWMThrottle);
    // SRV_Channels::get_output_pwm(SRV_Channel::k_rudder, PWMRudder);
    // // Set PWM outputs:
    // SRV_Channels::set_output_pwm(SRV_Channel::k_aileron, PWMAileron);
    // SRV_Channels::set_output_pwm(SRV_Channel::k_elevator, PWMElevator);
    // SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, PWMThrottle);
    // SRV_Channels::set_output_pwm(SRV_Channel::k_rudder, PWMRudder);
}

bool ModeAI_Deflection::is_AI_control() const
{
    return true;
}

void ModeAI_Deflection::handleMessage(const mavlink_execute_surface_deflection_override_t &packet)
{
    mapToDeflectionMixed(plane.channel_pitch, packet.deflection_elevator, 1, 2, false, _servoOutput.stickElevator);
    mapToDeflectionMixed(plane.channel_rudder, packet.deflection_rudder, 1, 2, false, _servoOutput.stickRudder);

    mapToDeflectionMixed(plane.channel_roll, packet.deflection_aileron, 1, 2, false, _servoOutput.stickAileron);

    //mapToDeflectionMixed(plane.channel_throttle, packet.deflection_throttle, 0, 1, false, _servoOutput.stickThrottle);
    _servoOutput.stickThrottle = 100 * packet.deflection_throttle;

    // printf("Throttle deflection (1): \n%f / %f\n", packet.deflection_throttle, _servoOutput.stickThrottle);

    update();

    uint16_t PWMAileron, PWMElevator, PWMThrottle, PWMRudder;

    SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, PWMAileron);
    SRV_Channels::get_output_pwm(SRV_Channel::k_elevator, PWMElevator);
    SRV_Channels::get_output_pwm(SRV_Channel::k_throttle, PWMThrottle);
    SRV_Channels::get_output_pwm(SRV_Channel::k_rudder, PWMRudder);

    AP::logger().WriteAI(AP_HAL::micros64(), packet.deflection_aileron, PWMAileron,
                         packet.deflection_elevator, PWMElevator,
                         packet.deflection_rudder, PWMRudder,
                         packet.deflection_throttle, PWMThrottle);

    int16_t PWMAileron1 = 0, PWMElevator1 = 0, PWMThrottle1 = 0, PWMRudder1 = 0;

    mapToDeflection(plane.channel_pitch, packet.deflection_elevator, 1, 2, false, PWMElevator1);
    mapToDeflection(plane.channel_rudder, packet.deflection_rudder, 1, 2, false, PWMRudder1);
    mapToDeflection(plane.channel_roll, packet.deflection_aileron, 1, 2, false, PWMAileron1);
    mapToDeflection(plane.channel_throttle, packet.deflection_throttle, 0, 1, false, PWMThrottle1);

    // printf("Servo output (1): \n%d / %d / %d / %d\n", PWMAileron1, PWMElevator1, PWMThrottle1, PWMRudder1);
    // printf("Servo output: \n%d / %d / %d / %d\n", PWMAileron, PWMElevator, PWMThrottle, PWMRudder);

}


bool ModeAI_Deflection::handleLongCommand(const mavlink_command_long_t &packet)
{
    if(packet.command != SET_SURFACE_DEFLECTION_NORMALIZED)
        return false;

    //printf("We are going to handle the command.");
    mapToDeflectionMixed(plane.channel_pitch, packet.param2, 1, 2, false, _servoOutput.stickElevator);
    mapToDeflectionMixed(plane.channel_rudder, packet.param4, 1, 2, false, _servoOutput.stickRudder);

    mapToDeflectionMixed(plane.channel_roll, packet.param3, 1, 2, false, _servoOutput.stickAileron);
    mapToDeflectionMixed(plane.channel_throttle, packet.param5, 0, 1, false, _servoOutput.stickThrottle);

    update();
    
    //take the params and map them to the servos
    return true;
}

void ModeAI_Deflection::mapToDeflection(RC_Channel *c, const float &value_in, const uint16_t &offset, const float &scaler, const bool &reversed, int16_t &servoValue)
{
    if (c == nullptr) {
        printf("        ****    NULLPTR     ****");
        return;
    }
    float currentValue = value_in;

    if (value_in != INT16_MAX) {
        const int16_t radio_min = c->get_radio_min();
        const int16_t radio_max = c->get_radio_max();
        if (reversed) {
            currentValue *= -1;
        }
    servoValue = (float)radio_min + ((float)radio_max - (float)radio_min) * ((float)currentValue + (float)offset) / scaler;
    
    }
}

void ModeAI_Deflection::mapToDeflectionMixed(RC_Channel *c, const float &value_in, const uint16_t &offset, const float &scaler, const bool &reversed, float &servoValue)
{
    if (c == nullptr) {
        printf("        ****    NULLPTR     ****");
        return;
    }
    float currentValue = value_in;

    if (value_in != INT16_MAX) {
        const int16_t radio_min = -4500 * offset;
        const int16_t radio_max = 4500;
        if (reversed) {
            currentValue *= -1;
        }
    servoValue = (float)radio_min + ((float)radio_max - (float)radio_min) * ((float)currentValue + (float)offset) / scaler;
    
    }
}


