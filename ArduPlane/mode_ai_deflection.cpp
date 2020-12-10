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
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, _servoOutput.stickAileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, _servoOutput.stickElevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _servoOutput.stickThrottle);

    plane.steering_control.steering = plane.steering_control.rudder = _servoOutput.stickRudder;
    AP::logger().WriteAI(AP_HAL::micros64(), 1, 1000);
}

bool ModeAI_Deflection::is_AI_control() const
{
    return true;
}

bool ModeAI_Deflection::handleLongCommand(const mavlink_command_long_t &packet)
{
    if(packet.command != SET_SURFACE_DEFLECTION_NORMALIZED)
        return false;

    printf("We are going to handle the command.");
    mapToDeflection(plane.channel_roll, packet.param2, 1, 2, false, _servoOutput.stickAileron);
    mapToDeflection(plane.channel_pitch, packet.param3, 1, 2, false, _servoOutput.stickElevator);
    mapToDeflection(plane.channel_throttle, packet.param4, 1, 2, false, _servoOutput.stickThrottle);
    mapToDeflection(plane.channel_rudder, packet.param5, 1, 2, false, _servoOutput.stickRudder);

    update();
    
    //take the params and map them to the servos
    return true;
}

void ModeAI_Deflection::mapToDeflection(RC_Channel *c, const int16_t &value_in, const uint16_t &offset, const float &scaler, const bool &reversed, int16_t &servoValue)
{
    if (c == nullptr) {
        return;
    }
    int16_t currentValue = value_in;

    if (value_in != INT16_MAX) {
        const int16_t radio_min = c->get_radio_min();
        const int16_t radio_max = c->get_radio_max();
        if (reversed) {
            currentValue *= -1;
        }
    servoValue = radio_min + (radio_max - radio_min) * (currentValue + offset) / scaler;
    
    }
}

