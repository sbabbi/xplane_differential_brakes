
#include <cstring>

#include <XPLMPlugin.h>
#include <XPLMDataAccess.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

// Switch betweeen version that uses the left_brake and right_brake command,
// and version that writes to the dataref directly
#if 0
static XPLMCommandRef left_brake = nullptr,
                      right_brake = nullptr,
                      both_brakes = nullptr;

static XPLMDataRef yaw_position = nullptr;

static int BothBrakesCommand( XPLMCommandRef inCommand,
                              XPLMCommandPhase inPhase,
                              void * inRefCon )
{
    static bool left_on = false, right_on = false;
    bool need_left = false, need_right = false;

    float yaw = XPLMGetDataf(yaw_position);

    switch (inPhase)
    {
    case xplm_CommandBegin:
    case xplm_CommandContinue:
        need_left = (yaw < 0.9);
        need_right = (yaw > -0.9);
        break;
    case xplm_CommandEnd:
        need_left = need_right = false;
        break;
    }

    if ( need_left && !left_on )
        XPLMCommandButtonPress( xplm_joy_lft_brake );
    if ( !need_left && left_on )
        XPLMCommandButtonRelease( xplm_joy_lft_brake );

    if ( need_right && !right_on )
        XPLMCommandButtonPress( xplm_joy_rgt_brake );
    if ( !need_right && right_on )
        XPLMCommandButtonRelease( xplm_joy_rgt_brake );

    left_on = need_left, right_on = need_right;

    return 0;
}

PLUGIN_API int XPluginStart ( char * outName, char * outSignature, char * outDescription ) {

    left_brake = XPLMFindCommand("sim/flight_controls/left_brake");
    right_brake = XPLMFindCommand("sim/flight_controls/right_brake");
    both_brakes = XPLMCreateCommand("sim/flight_controls/both_brakes", "Brake left and right, differential brake depending on the rudder input");

    yaw_position = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");

    XPLMRegisterCommandHandler( both_brakes,
                                BothBrakesCommand,
                                0,
                                (void*)0);

    if (left_brake && right_brake && both_brakes && yaw_position) {
        strcpy(outName, "differential_brakes");
        strcpy(outSignature, "differential_brakes by Ennio Barbaro");
        strcpy(outDescription, "Apply left and/or right brake depending on joystick yaw input.");
        return 1;
    }
    return 0;
}

PLUGIN_API void XPluginStop ( void ) {
    XPLMUnregisterCommandHandler( both_brakes,
                                BothBrakesCommand,
                                0,
                                (void*)0);
}

PLUGIN_API void XPluginEnable ( void ) {
}

PLUGIN_API void XPluginDisable ( void ) {
    XPLMCommandButtonRelease( xplm_joy_lft_brake );
    XPLMCommandButtonRelease( xplm_joy_rgt_brake );
}

#else

static XPLMCommandRef both_brakes = nullptr;
static XPLMDataRef yaw_position = nullptr,
                    left_brake_ratio = nullptr,
                    right_brake_ratio = nullptr;

static int BothBrakesCommand( XPLMCommandRef inCommand,
                              XPLMCommandPhase inPhase,
                              void * inRefCon )
{
    auto filter = [](float yaw)
    {
        const float differential_brake_limit = 0.8;
        return yaw < differential_brake_limit ?
                1.0 :
                (1.0 - yaw) / (1.0 - differential_brake_limit);
    };

    const float yaw = XPLMGetDataf(yaw_position);
    const float left_press = filter(yaw);
    const float right_press = filter(-yaw);

    switch (inPhase)
    {
    case xplm_CommandBegin:
    case xplm_CommandContinue:
        XPLMSetDataf( left_brake_ratio, left_press );
        XPLMSetDataf( right_brake_ratio, right_press );
        break;
    case xplm_CommandEnd:
        XPLMSetDataf( left_brake_ratio, 0.0 );
        XPLMSetDataf( right_brake_ratio, 0.0 );
        break;
    }

    return 0;
}

PLUGIN_API int XPluginStart ( char * outName, char * outSignature, char * outDescription ) {

    both_brakes = XPLMCreateCommand("sim/flight_controls/both_brakes", "Brake left and right, differential brake depending on the rudder input");

    left_brake_ratio = XPLMFindDataRef("sim/cockpit2/controls/left_brake_ratio");
    right_brake_ratio = XPLMFindDataRef("sim/cockpit2/controls/right_brake_ratio");
    yaw_position = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");

    XPLMRegisterCommandHandler( both_brakes,
                                BothBrakesCommand,
                                0,
                                (void*)0);

    if (both_brakes && left_brake_ratio && right_brake_ratio && yaw_position) {
        strcpy(outName, "differential_brakes");
        strcpy(outSignature, "differential_brakes by Ennio Barbaro");
        strcpy(outDescription, "Apply left and/or right brake depending on joystick yaw input.");
        return 1;
    }
    return 0;
}

PLUGIN_API void XPluginStop ( void ) {
    XPLMUnregisterCommandHandler( both_brakes,
                                BothBrakesCommand,
                                0,
                                (void*)0);
}

PLUGIN_API void XPluginEnable ( void ) {
}

PLUGIN_API void XPluginDisable ( void ) {
    XPLMCommandButtonRelease( xplm_joy_lft_brake );
    XPLMCommandButtonRelease( xplm_joy_rgt_brake );
}

#endif
