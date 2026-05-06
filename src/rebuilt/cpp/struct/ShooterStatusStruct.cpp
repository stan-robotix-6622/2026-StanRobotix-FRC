// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "rebuilt/struct/ShooterStatusStruct.h"

namespace
{
	constexpr size_t kDistanceOff = 0;
	constexpr size_t kVelocityOff = kDistanceOff + 8;
	constexpr size_t kTOFOff = kVelocityOff + 8;
} // namespace

using StructType = wpi::Struct<LookupTable::ShooterStatus>;

LookupTable::ShooterStatus StructType::Unpack(std::span<const uint8_t> data)
{
	return LookupTable::ShooterStatus{
		units::meter_t{wpi::UnpackStruct<double, kDistanceOff>(data)},
		units::turns_per_second_t{wpi::UnpackStruct<double, kVelocityOff>(data)},
		units::second_t{wpi::UnpackStruct<double, kTOFOff>(data)},
	};
}

void StructType::Pack(std::span<uint8_t> data, const LookupTable::ShooterStatus& value)
{
	wpi::PackStruct<kDistanceOff>(data, value.distanceToTarget.value());
	wpi::PackStruct<kVelocityOff>(data, value.shooterVelocity.value());
	wpi::PackStruct<kTOFOff>(data, value.timeOfFlight.value());
}
