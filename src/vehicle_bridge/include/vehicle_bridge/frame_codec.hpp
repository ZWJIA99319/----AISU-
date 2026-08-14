#pragma once
#include <array>
#include <cstdint>
namespace frame_codec {
struct SensorFrame{double vx,vy,wz,ax,ay,az,gx,gy,gz,voltage;};
std::array<uint8_t,11> encode_velocity(double x,double y,double yaw);
bool decode_sensor(const std::array<uint8_t,24>& bytes,SensorFrame& value);
}
