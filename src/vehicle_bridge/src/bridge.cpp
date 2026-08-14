#include "vehicle_bridge/frame_codec.hpp"
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/float32.hpp"
using namespace std::chrono_literals;
class VehicleBridge:public rclcpp::Node{
 serial::Serial link; rclcpp::TimerBase::SharedPtr reader; rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command;
 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry; rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr inertial; rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery;
 void transmit(double x,double y,double z){if(!link.isOpen())return;auto bytes=frame_codec::encode_velocity(x,y,z);try{link.write(bytes.data(),bytes.size());}catch(const std::exception& e){RCLCPP_WARN(get_logger(),"发送失败: %s",e.what());}}
 void receive(){if(!link.isOpen()||link.available()<24)return;std::array<uint8_t,24> bytes{};if(link.read(bytes.data(),24)!=24)return;frame_codec::SensorFrame s;if(!frame_codec::decode_sensor(bytes,s))return;auto timestamp=now();
  nav_msgs::msg::Odometry od;od.header.stamp=timestamp;od.header.frame_id="odom";od.child_frame_id="base_footprint";od.pose.pose.orientation.w=1.;od.pose.covariance.fill(1e6);od.twist.covariance.fill(1e6);od.twist.covariance[0]=1e-4;od.twist.covariance[7]=1e-3;od.twist.twist.linear.x=s.vx;od.twist.twist.linear.y=s.vy;odometry->publish(od);
  sensor_msgs::msg::Imu im;im.header.stamp=timestamp;im.header.frame_id="gyro_link";im.orientation_covariance[0]=-1;im.angular_velocity.x=s.gx;im.angular_velocity.y=s.gy;im.angular_velocity.z=s.gz;im.linear_acceleration.x=s.ax;im.linear_acceleration.y=s.ay;im.linear_acceleration.z=s.az;im.angular_velocity_covariance[0]=im.angular_velocity_covariance[4]=1e6;im.angular_velocity_covariance[8]=1e-3;im.linear_acceleration_covariance[0]=im.linear_acceleration_covariance[4]=im.linear_acceleration_covariance[8]=1e-2;inertial->publish(im);
  std_msgs::msg::Float32 v;v.data=s.voltage;battery->publish(v);
 }
public:VehicleBridge():Node("vehicle_bridge"){
 declare_parameter("serial_device","/dev/ttyACM0");declare_parameter("serial_baud",115200);odometry=create_publisher<nav_msgs::msg::Odometry>("odom",10);inertial=create_publisher<sensor_msgs::msg::Imu>("imu/data_raw",10);battery=create_publisher<std_msgs::msg::Float32>("PowerVoltage",1);
 command=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",rclcpp::QoS(1).best_effort(),[this](geometry_msgs::msg::Twist::SharedPtr m){transmit(m->linear.x,m->linear.y,m->angular.z);});
 try{link.setPort(get_parameter("serial_device").as_string());link.setBaudrate(get_parameter("serial_baud").as_int());link.setTimeout(serial::Timeout::simpleTimeout(2000));link.open();}catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"无法连接底盘: %s",e.what());}reader=create_wall_timer(2ms,[this]{receive();});}
 ~VehicleBridge(){transmit(0,0,0);if(link.isOpen())link.close();}
};
int main(int argc,char* argv[]){rclcpp::init(argc,argv);auto node=std::make_shared<VehicleBridge>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;}
