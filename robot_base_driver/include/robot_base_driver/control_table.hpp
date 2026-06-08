#pragma once

#include <cstdint>

namespace robot::hw::base
{

struct ControlItem
{
	uint16_t address;
	uint8_t memory;
	uint16_t length;
	uint8_t access;
};

class ControlTable
{
private:
protected:
public:
	static constexpr uint8_t EEPROM = 1;
	static constexpr uint8_t RAM = 2;

	static constexpr uint8_t READ = 1;
	static constexpr uint8_t READ_WRITE = 3;

	static constexpr uint16_t OPENCR_ID = 200;

	static constexpr ControlItem MILLIS = {10, RAM, 4, READ};
	static constexpr ControlItem DEVICE_STATUS = {18, RAM, 1, READ};
	static constexpr ControlItem HEARTBEAT = {19, RAM, 1, READ_WRITE};
	static constexpr ControlItem BATTERY_VOLTAGE = {42, RAM, 4, READ};
	static constexpr ControlItem BATTERY_PERCENTAGE = {46, RAM, 4, READ};
	static constexpr ControlItem IMU_RECALIBRATION = {59, RAM, 1, READ_WRITE};
	static constexpr ControlItem IMU_ANGULAR_VELOCITY_X = {60, RAM, 4, READ};
	static constexpr ControlItem IMU_ANGULAR_VELOCITY_Y = {64, RAM, 4, READ};
	static constexpr ControlItem IMU_ANGULAR_VELOCITY_Z = {68, RAM, 4, READ};
	static constexpr ControlItem IMU_LINEAR_ACCELERATION_X = {72, RAM, 4, READ};
	static constexpr ControlItem IMU_LINEAR_ACCELERATION_Y = {76, RAM, 4, READ};
	static constexpr ControlItem IMU_LINEAR_ACCELERATION_Z = {80, RAM, 4, READ};
	static constexpr ControlItem IMU_MAGNETIC_X = {84, RAM, 4, READ};
	static constexpr ControlItem IMU_MAGNETIC_Y = {88, RAM, 4, READ};
	static constexpr ControlItem IMU_MAGNETIC_Z = {92, RAM, 4, READ};
	static constexpr ControlItem IMU_ORIENTATION_W = {96, RAM, 4, READ};
	static constexpr ControlItem IMU_ORIENTATION_X = {100, RAM, 4, READ};
	static constexpr ControlItem IMU_ORIENTATION_Y = {104, RAM, 4, READ};
	static constexpr ControlItem IMU_ORIENTATION_Z = {108, RAM, 4, READ};
	static constexpr ControlItem PRESENT_CURRENT_LEFT = {120, RAM, 4, READ};
	static constexpr ControlItem PRESENT_CURRENT_RIGHT = {124, RAM, 4, READ};
	static constexpr ControlItem PRESENT_VELOCITY_LEFT = {128, RAM, 4, READ};
	static constexpr ControlItem PRESENT_VELOCITY_RIGHT = {132, RAM, 4, READ};
	static constexpr ControlItem PRESENT_POSITION_LEFT = {136, RAM, 4, READ};
	static constexpr ControlItem PRESENT_POSITION_RIGHT = {140, RAM, 4, READ};
	static constexpr ControlItem MOTOR_TORQUE_ENABLE = {149, RAM, 1, READ_WRITE};
	static constexpr ControlItem CMD_VELOCITY_LINEAR_X = {150, RAM, 4, READ_WRITE};
	static constexpr ControlItem CMD_VELOCITY_LINEAR_Y = {154, RAM, 4, READ_WRITE};
	static constexpr ControlItem CMD_VELOCITY_LINEAR_Z = {158, RAM, 4, READ_WRITE};
	static constexpr ControlItem CMD_VELOCITY_ANGULAR_X = {162, RAM, 4, READ_WRITE};
	static constexpr ControlItem CMD_VELOCITY_ANGULAR_Y = {166, RAM, 4, READ_WRITE};
	static constexpr ControlItem CMD_VELOCITY_ANGULAR_Z = {170, RAM, 4, READ_WRITE};
	static constexpr ControlItem PROFILE_ACCELERATION_LEFT = {174, RAM, 4, READ_WRITE};
	static constexpr ControlItem PROFILE_ACCELERATION_RIGHT = {178, RAM, 4, READ_WRITE};

	static constexpr uint16_t READ_START_ADDRESS = MILLIS.address;
	static constexpr uint16_t READ_BLOCK_LENGTH =
		(PROFILE_ACCELERATION_RIGHT.address - MILLIS.address) + PROFILE_ACCELERATION_RIGHT.length;
};

}  // namespace robot::hw::base
