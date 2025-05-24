/* main.cpp  –  d6 = hard stop, d2 = 90° spin then pivot-FSM
 *
 * drive_mode
 *   0 : raw joystick
 *   1 : joystick + obstacle brakes
 *   2 : autonomous explained above
 */
 
#include "main.h"
#include "uwb_subscriber.hpp"  
#include "rclcpp/rclcpp.hpp"
#include "sensors_subscriber.hpp"
#include "obstacle_subscriber.hpp"
#include "ref_speed_publisher.hpp"
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <thread>

/* ── constants ─────────────────────────────────────────── */
constexpr int    SPEED           = 30;
constexpr int    INNER_SPEED     = SPEED / 4;
constexpr double PIVOT_TIME      = 2.5;
constexpr double RETURN_TIME     = 2.5;
constexpr float  UWB_STOP_RANGE  = 2.15;   // d6 ≤ 2.15 m → hard stop
constexpr float  UWB_TURN_RANGE  = 3.0;   // d2 ≤ 3.0  m → spin + FSM
constexpr double TURN90_TIME     = 3.2;     


/* ── shared flags set by subscribers ───────────────────── */
std::atomic_bool front_clear{true};
std::atomic_bool back_clear{true};
std::atomic_bool left_turn_clear{true};
std::atomic_bool right_turn_clear{true};
std::atomic_int  drive_mode{0};

/* ── FSM state for complex part ────────────────────────── */
enum class Mode { STRAIGHT, PIVOT, RETURN, STOP };
Mode   mode   = Mode::STRAIGHT;
double timer  = 0.0;
int    pivot_dir = -1;          // −1 = first swing left

/* ── helper for one-shot 90° turn on d2 trigger ────────── */
bool   turning90 = false;
double turn_t    = 0.0;
bool   after_spin = false;      // ensures we enter FSM right after spin
float angle = 0.0;
const float target_angle = 90.0;

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
        
    /* receive message from UWB sensors */
    auto uwb_sub = std::make_shared<UWBSubscriber>(node);
    

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
            bool  l_ok = left_turn_clear.load();
            float d6   = uwb_sub->dist6();      // front beacon
            float d2   = uwb_sub->dist2();      // left-flank sensor
            
           RCLCPP_INFO(node->get_logger(),
              "UWB2: %f", d2);
           
            RefSpeed cmd{SPEED, SPEED};
            ref_pub->trigger_publish(cmd);
            



            /* d6 beacon → hard stop */
            if (d6 > 0.f && d6 < UWB_STOP_RANGE)
            {
                cmd = {0,0};
                ref_pub->trigger_publish(cmd);  loop.sleep();  continue;
            }

            /* d2 trigger → one 90° spin (left) then hand off to FSM */
            if (d2 > 0 && d2 < UWB_TURN_RANGE && turning90 == false)
            {
                turning90 = true;
                spin_start = steady_clock.now();
                turn_t = 0.0;
                RCLCPP_INFO(node->get_logger(), "Starting turn");
                
            }
            
            if(turning90 == true  && !after_spin){
                auto sensorValue = sensors_sub->get_latest_sensor_data();
                float gyroValue = sensorValue.angular_velocity_z * (180.0f / M_PI);
            	RCLCPP_INFO(node->get_logger(), "Turning");
            	if (abs(angle) < target_angle) {
                    angle += gyroValue * DT;
                    RefSpeed cmd{0, SPEED};
                    ref_pub->trigger_publish(cmd); 
                    RCLCPP_INFO(node->get_logger(),
                    "Turning… angle = %.3f deg / %.3f deg",
                    angle, target_angle);
                    loop.sleep();  
                    continue;
                }
                /* spin finished */
                //turning90 = false; 
                after_spin = true;
                cmd = {0,0};
                ref_pub->trigger_publish(cmd);  loop.sleep();  continue;
                
            }
           

            /* After the spin, immediately run the complex FSM */
            if (after_spin)
            {
                switch (mode)
                {
                case Mode::STRAIGHT:
                    if (!f_ok) {
                        if (!l_ok)          mode = Mode::STOP;
                        else { pivot_dir = -1; timer = 0; mode = Mode::PIVOT; }
                    }
                    break;

                case Mode::PIVOT:
                    if (!l_ok) { mode = Mode::STOP; break; }
                    timer += DT;
                    cmd.leftSpeed  = (pivot_dir == +1) ? SPEED       : INNER_SPEED;
                    cmd.rightSpeed = (pivot_dir == +1) ? INNER_SPEED : SPEED;
                    if (timer >= PIVOT_TIME) {
                        pivot_dir = -pivot_dir;
                        timer = 0;  mode = Mode::RETURN;
                    }
                    break;

                case Mode::RETURN:
                    if (!l_ok) { mode = Mode::STOP; break; }
                    timer += DT;
                    cmd.leftSpeed  = (pivot_dir == +1) ? SPEED       : INNER_SPEED;
                    cmd.rightSpeed = (pivot_dir == +1) ? INNER_SPEED : SPEED;
                    if (timer >= RETURN_TIME) {
                        timer = 0;  mode = f_ok ? Mode::STRAIGHT : Mode::STOP;
                    }
                    break;

                case Mode::STOP:
                    cmd = {0,0};
                    if (f_ok) mode = Mode::STRAIGHT;
                    break;
                }

                ref_pub->trigger_publish(cmd);  loop.sleep();  continue;
            }

            /* If none of the above, just keep going straight */
            ref_pub->trigger_publish(cmd);  loop.sleep();  continue;
        }

        loop.sleep();     // unknown drive_mode
    }

    exec.cancel();  spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
