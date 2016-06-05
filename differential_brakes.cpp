
#include <algorithm>
#include <cstring>
#include <cmath>

#ifdef DIFF_BRAKES_TEST
#include <iostream>
#else
#include <XPLMPlugin.h>
#include <XPLMDataAccess.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#endif //DIFF_BRAKES_TEST

namespace
{
	/**
	 * Try to smooth the brake command.
	 */
	class brake_filter
	{
	public:
		float step(bool signal, float dt)
		{
			if (prev_s_ != signal)
			{
				int_e_ = 0.0f;
				prev_s_ = signal;
			}

			const float e =  (signal ? 1.1f : -0.1f) - x_;
			const float de = (e - prev_e_) / dt;
			prev_e_ = e;
			int_e_ += e * dt;

			const float Kp = 0.4f;
			const float Ki = 2.0f;
			const float Kd = 0.01f;

			return x_ = std::max(0.0f, std::min(1.0f,  x_ + (Kp * e + Ki * int_e_ + Kd * de) * dt ) );
		}

	private:
		float x_ = 0.0f;
		float prev_e_ = 0.0f;
		float int_e_ = 0.0f;
		bool prev_s_ = false;
	};

	/**
	 * Partition the braking action between left and right, depending on the yaw input
	 */
	float differential_factor(float yaw)
	{
		const float differential_brake_limit = 0.8f;
		return yaw < differential_brake_limit ?
			1.0f :
			(1.0f - yaw) / (1.0f - differential_brake_limit);
	}
}

#ifndef DIFF_BRAKES_TEST
namespace 
{
	XPLMCommandRef both_brakes = nullptr;
	XPLMDataRef yaw_position = nullptr,
				left_brake_ratio = nullptr,
				right_brake_ratio = nullptr;

	bool flight_loop_active = false;
	bool brake_button_pressed = false;
	brake_filter filter;
	XPLMFlightLoopID flight_loop_id;

	int BothBrakesCommand(XPLMCommandRef inCommand,
						  XPLMCommandPhase inPhase,
						  void * inRefCon)
	{
		switch (inPhase)
		{
		case xplm_CommandBegin:
			brake_button_pressed = true;
			if (!flight_loop_active)
			{
				XPLMScheduleFlightLoop(flight_loop_id, 0.1f, 1);
				flight_loop_active = true;
			}
			break;
		case xplm_CommandContinue:
			break;
		case xplm_CommandEnd:
			brake_button_pressed = false;
			break;
		}

		return 0;
	}

	float FlightLoop(float inElapsedSinceLastCall,
					 float /*inElapsedTimeSinceLastFlightLoop*/,
					 int  /*inCounter*/,
					 void * /*inRefcon*/)
	{
		const float yaw = XPLMGetDataf(yaw_position);
		const float left_press = differential_factor(yaw);
		const float right_press = differential_factor(-yaw);

		const float brake_action = filter.step(brake_button_pressed, inElapsedSinceLastCall);

		XPLMSetDataf(left_brake_ratio, left_press * brake_action);
		XPLMSetDataf(right_brake_ratio, right_press * brake_action);

		if (brake_action <= 0.0f && !brake_button_pressed)
		{
			flight_loop_active = false;
			return 0.0f;
		}
		return 0.1f;
	}
}

PLUGIN_API int XPluginStart ( char * outName, char * outSignature, char * outDescription ) {

    both_brakes = XPLMCreateCommand("sim/flight_controls/both_brakes", "Brake left and right, differential brake depending on the rudder input");

    left_brake_ratio = XPLMFindDataRef("sim/cockpit2/controls/left_brake_ratio");
    right_brake_ratio = XPLMFindDataRef("sim/cockpit2/controls/right_brake_ratio");
    yaw_position = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");

	XPLMCreateFlightLoop_t flight_loop_info = 
	{
		sizeof(XPLMCreateFlightLoop_t),
		xplm_FlightLoop_Phase_BeforeFlightModel,
		FlightLoop,
		nullptr
	};

	flight_loop_id = XPLMCreateFlightLoop(&flight_loop_info);

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

	XPLMDestroyFlightLoop(flight_loop_id);
	flight_loop_active = false;
}

PLUGIN_API void XPluginEnable ( void ) {
}

PLUGIN_API void XPluginDisable ( void ) {
    XPLMCommandButtonRelease( xplm_joy_lft_brake );
    XPLMCommandButtonRelease( xplm_joy_rgt_brake );
}


#ifdef _MSC_VER
#include <windows.h>
BOOL APIENTRY DllMain(HANDLE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}
#endif //_MSC_VER

#else  // DIFF_BRAKES_TEST

int main()
{
	auto rect_signal = [](float t) -> bool
	{
		const float period = 2.0f;

		return fmod( fabs(t), period) < period * 0.75f;
	};
	
	brake_filter filter;

	const float dt = 0.1f;
	for (float t = 0.0f; t < 10.0f; t += dt)
	{
		float x = filter.step(rect_signal(t), dt);
		std::cout << "s = " << rect_signal(t) << " t = " << t << " brake = " << x << std::endl;
	}

#ifdef _WIN32
	system("PAUSE");
#endif //_WIN32
}

#endif //
