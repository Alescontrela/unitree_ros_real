/**
 * @file sdl_gamepad.cpp
 * @brief Implementation of SdlGamepad ROS node.
 */

#include "sdl_gamepad.h"

SdlGamepad::SdlGamepad(ros::NodeHandle _nh)
: nh(_nh)
{
    get_params();

    joy_pub = nh.advertise<sensor_msgs::Joy>("joy", 10);

    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        ROS_FATAL("SDL_Init failed: %s", SDL_GetError());
        throw std::runtime_error("SDL init failure");
    }

    if (!open_first_controller()) {
        ROS_FATAL("No SDL2-compatible controller found");
        throw std::runtime_error("No controller");
    }

    axes.resize(SDL_CONTROLLER_AXIS_MAX, 0.0f);
    buttons.resize(SDL_CONTROLLER_BUTTON_MAX, 0);

    last_publish_time = ros::Time(0);

    main_loop_timer = nh.createTimer(
        ros::Duration(1.0 / rate_hz),
        &SdlGamepad::timer_cb,
        this
    );

    ROS_INFO("sdl_gamepad node started (rate=%d Hz, deadzone=%.3f)", rate_hz, deadzone);
}

SdlGamepad::~SdlGamepad()
{
    close_controller();
    SDL_Quit();
}

void SdlGamepad::get_params()
{
    nh.param("rate_hz", rate_hz, 100);
    nh.param("deadzone", deadzone, 0.05f);
    nh.param("auto_repeat", auto_repeat, true);
    nh.param("repeat_timeout", repeat_timeout, 0.1);
}

bool SdlGamepad::open_first_controller()
{
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            ctrl = SDL_GameControllerOpen(i);
            if (ctrl) {
                ctrl_instance = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(ctrl));
                ROS_INFO("Opened controller: %s", SDL_GameControllerName(ctrl));
                return true;
            }
        }
    }
    return false;
}

void SdlGamepad::close_controller()
{
    if (ctrl) {
        SDL_GameControllerClose(ctrl);
        ctrl = nullptr;
        ctrl_instance = -1;
    }
}

static inline float normalize_axis(Sint16 v)
{
    return (v >= 0) ? (v / 32767.0f) : (v / 32768.0f);
}

void SdlGamepad::poll_sdl_events()
{
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        switch (e.type) {
        case SDL_CONTROLLERAXISMOTION: {
            if (e.caxis.which != ctrl_instance) break;
            float v = normalize_axis(e.caxis.value);
            if (std::abs(v) < deadzone) v = 0.0f;
            if (e.caxis.axis < axes.size()) axes[e.caxis.axis] = v;
        } break;
        case SDL_CONTROLLERBUTTONDOWN:
        case SDL_CONTROLLERBUTTONUP: {
            if (e.cbutton.which != ctrl_instance) break;
            int val = (e.type == SDL_CONTROLLERBUTTONDOWN) ? 1 : 0;
            if (e.cbutton.button < buttons.size()) buttons[e.cbutton.button] = val;
        } break;
        case SDL_CONTROLLERDEVICEADDED:
            // If we had no controller, try open
            if (!ctrl) open_first_controller();
            break;
        case SDL_CONTROLLERDEVICEREMOVED:
            if (e.cdevice.which == ctrl_instance) {
                ROS_WARN("Controller removed");
                close_controller();
            }
            break;
        default: break;
        }
    }
}

void SdlGamepad::timer_cb(const ros::TimerEvent&)
{
    if (!ros::ok()) return;

    poll_sdl_events();

    // If controller gone, skip publishing (or could attempt reopen periodically)
    if (!ctrl) return;

    ros::Time now = ros::Time::now();
    bool time_to_repeat = auto_repeat && (now - last_publish_time).toSec() >= repeat_timeout;

    // Always publish on timer if auto_repeat, or only when changed?
    // For simplicity, always publish on timer; deadzone keeps noise down.
    if (time_to_repeat || auto_repeat || true) {
        sensor_msgs::Joy msg;
        msg.header.stamp = now;
        msg.axes = axes;
        msg.buttons = buttons;
        joy_pub.publish(msg);
        last_publish_time = now;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sdl_gamepad");
    ros::NodeHandle nh("~");

    try {
        SdlGamepad node(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Exception: %s", e.what());
        return 1;
    }
    return 0;
}
