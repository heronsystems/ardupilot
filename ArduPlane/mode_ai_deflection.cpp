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
    
    _servoOutput.stickAileron = plane.channel_roll->get_control_in();
    _servoOutput.stickElevator = plane.channel_pitch->get_control_in();
    _servoOutput.stickThrottle = plane.channel_throttle->get_control_in();
    _servoOutput.stickRudder = plane.channel_rudder->get_control_in();

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
    SRV_Channels::set_output_pwm(SRV_Channel::k_aileron, _servoOutput.stickAileron);
    SRV_Channels::set_output_pwm(SRV_Channel::k_elevator, _servoOutput.stickElevator);
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, _servoOutput.stickThrottle);

    plane.steering_control.steering = plane.steering_control.rudder = _servoOutput.stickRudder;
}

bool ModeAI_Deflection::is_AI_control() const
{
    return true;
}

void ModeAI_Deflection::handleMessage(const mavlink_execute_surface_deflection_override_t &packet)
{
    mapToDeflection(plane.channel_pitch, packet.deflection_elevator, 1, 2, false, _servoOutput.stickElevator);
    mapToDeflection(plane.channel_roll, packet.deflection_aileron, 1, 2, false, _servoOutput.stickAileron);
    mapToDeflection(plane.channel_rudder, packet.deflection_rudder, 1, 2, false, _servoOutput.stickRudder);
    mapToDeflection(plane.channel_throttle, packet.deflection_throttle, 0, 1, false, _servoOutput.stickThrottle);

    AP::logger().WriteAI(AP_HAL::micros64(), packet.deflection_elevator, _servoOutput.stickElevator,
                         packet.deflection_aileron, _servoOutput.stickAileron,
                         packet.deflection_rudder, _servoOutput.stickRudder,
                         packet.deflection_throttle, _servoOutput.stickThrottle);

    update();
}


bool ModeAI_Deflection::handleLongCommand(const mavlink_command_long_t &packet)
{
    if(packet.command != SET_SURFACE_DEFLECTION_NORMALIZED)
        return false;

    printf("We are going to handle the command.");
    mapToDeflection(plane.channel_pitch, packet.param2, 1, 2, false, _servoOutput.stickElevator);
    mapToDeflection(plane.channel_roll, packet.param3, 1, 2, false, _servoOutput.stickAileron);
    mapToDeflection(plane.channel_rudder, packet.param4, 1, 2, false, _servoOutput.stickRudder);
    mapToDeflection(plane.channel_throttle, packet.param5, 0, 1, false, _servoOutput.stickThrottle);

    printf("\nPacket: \n%f / %f / %f / %f\n", packet.param2, packet.param3, packet.param4, packet.param5);
    printf("Servo output: \n%d / %d / %d / %d\n", _servoOutput.stickElevator, _servoOutput.stickAileron, _servoOutput.stickRudder, _servoOutput.stickThrottle);

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

