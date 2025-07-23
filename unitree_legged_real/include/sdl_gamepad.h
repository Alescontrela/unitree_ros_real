/**
 * @file sdl_gamepad.h
 * @author Alejandro Escontrela
 * @brief ROS node that publishes sensor_msgs/Joy using SDL2 GameController API.
 * @version 0.1
 * @date 2025-07-23
 */

#ifndef _SDL_GAMEPAD_NODE_H_
#define _SDL_GAMEPAD_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <SDL.h>
#include <vector>
#include <string>

class SdlGamepad
{
public:
    // ROS
    ros::NodeHandle nh;          // private "~"
    ros::Publisher  joy_pub;
    ros::Timer      main_loop_timer;

    // Params
    int   rate_hz = 100;
    float deadzone = 0.05f;
    bool  auto_repeat = true;    // publish even if nothing changed
    double repeat_timeout = 0.1; // seconds between repeats

    // SDL
    SDL_GameController* ctrl = nullptr;
    SDL_JoystickID      ctrl_instance = -1;

    // Buffers
    std::vector<float> axes;
    std::vector<int32_t> buttons;
    ros::Time last_publish_time;

protected:
    void get_params();
    bool open_first_controller();
    void close_controller();

    void poll_sdl_events(); // handles SDL events, updates buffers
    void timer_cb(const ros::TimerEvent&);

public:
    explicit SdlGamepad(ros::NodeHandle _nh);
    ~SdlGamepad();
};

#endif
