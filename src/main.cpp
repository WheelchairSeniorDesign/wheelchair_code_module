/* main.cpp 
 *
 * drive_mode
 *   0 : raw joystick
 *   1 : joystick + obstacle brakes
 *   2 : autonomous explained above
 */

#include "main.h"
#include "rclcpp/rclcpp.hpp"
#include "sensors_subscriber.hpp"
#include "obstacle_subscriber.hpp"
#include "ref_speed_publisher.hpp"
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <thread>
#include <cstdlib>

/* ── constants ─────────────────────────────────────────── */
constexpr int    SPEED           = 30;
constexpr int    INNER_SPEED     = 0;

/* ── shared flags set by subscribers ───────────────────── */
std::atomic_bool front_clear{true};
std::atomic_bool back_clear{true};
std::atomic_bool left_turn_clear{true};
std::atomic_bool right_turn_clear{true};
std::atomic_int  drive_mode{0};
std::atomic_int  room{0};

/* ─────────────────────────────────────────────────────── */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wheelchair_code_module");

    /* pubs / subs */
    auto sensors_sub  = std::make_shared<SensorsSubscriber>(node);
    auto obstacle_sub = std::make_shared<ObstacleSubscriber>(
        node, front_clear, back_clear, left_turn_clear, right_turn_clear);
    auto ref_pub      = std::make_shared<RefSpeedPublisher>(node);

    /* receive drive-mode commands from Electron GUI */
    auto electron_sub = node->create_subscription<std_msgs::msg::Int32>(
        "electron_selfdrive", 10,
        [node](std_msgs::msg::Int32::SharedPtr m)
        {
            drive_mode.store(m->data, std::memory_order_relaxed);
            RCLCPP_INFO(node->get_logger(), "[RX] drive-mode=%d", m->data);
        });


    /* spin ROS callbacks in a background thread */
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spin_thread([&]{ exec.spin(); });
    rclcpp::Clock steady_clock{RCL_STEADY_TIME};
    rclcpp::Time  spin_start;

    /* main control loop – 30 Hz */
    rclcpp::Rate loop(30);
    const double DT = 1.0 / 30.0;

    while (rclcpp::ok())
    {
        int sel = drive_mode.load();
        int room_val = room.load();
        RefSpeed cmd{SPEED, SPEED};

        /* ----------  drive_mode 0 : raw joystick  ---------- */

        if (sel == 0) {
            auto j = sensors_sub->get_latest_sensor_data();
            cmd.leftSpeed  = j.left_speed;
            cmd.rightSpeed = j.right_speed;
            ref_pub->trigger_publish(cmd);
            loop.sleep();
            continue;
        }

        /* ----------  drive_mode 1 : joystick + brakes  ----- */
        if (sel == 1) {

            auto j = sensors_sub->get_latest_sensor_data();
            cmd.leftSpeed  = j.left_speed;
            cmd.rightSpeed = j.right_speed;

            bool f_ok = front_clear.load(), b_ok = back_clear.load();
            if (!f_ok && cmd.leftSpeed  > 0) cmd.leftSpeed  = 0;
            if (!f_ok && cmd.rightSpeed > 0) cmd.rightSpeed = 0;
            if (!b_ok && cmd.leftSpeed  < 0) cmd.leftSpeed  = 0;
            if (!b_ok && cmd.rightSpeed < 0) cmd.rightSpeed = 0;

            ref_pub->trigger_publish(cmd);  loop.sleep();  continue;
        }

        /* ----------  drive_mode 2 : autonomous  ------------ */
        if (sel == 2)
        {

            bool  f_ok = front_clear.load();
            bool b_ok  = !back_clear.load(std::memory_order_relaxed);
            bool  l_ok = left_turn_clear.load();

            RefSpeed cmd{SPEED, SPEED};

            if (!f_ok && !b_ok) {                // both ways blocked
            	cmd = {0, 0};
            } else if (!f_ok) {                 // stop forward
            	if (cmd.leftSpeed  > 0) cmd.leftSpeed  = 0;
            	if (cmd.rightSpeed > 0) cmd.rightSpeed = 0;
            } else if (!b_ok) {                          // stop reverse
            	if (cmd.leftSpeed  < 0) cmd.leftSpeed  = 0;
            	if (cmd.rightSpeed < 0) cmd.rightSpeed = 0;
            }

            ref_pub->trigger_publish(cmd);
            loop.sleep();
            continue;
        }

        loop.sleep();     // unknown drive_mode
    }

    exec.cancel();  
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
