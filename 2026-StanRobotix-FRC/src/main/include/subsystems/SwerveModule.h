// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/system/plant/DCMotor.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

class SwerveModule : public wpi::Sendable {
 public:
	SwerveModule(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInveryed = false, bool iTurningInverted = true);

	frc::SwerveModulePosition getModulePosition();
	frc::SwerveModuleState getModuleState();

	units::radians_per_second_t getTurningVelocity();

	void InitSendable(wpi::SendableBuilder& builder) override;

	void setDesiredState(frc::SwerveModuleState iDesiredState);
	void setDesiredHeading(frc::Rotation2d iDesiredHeading);

	void setTurningVoltage(units::volt_t iVoltage);
	void setDrivingVoltage(units::volt_t iVoltage);

	void refreshModule();

 private:
	rev::spark::SparkMax* mDrivingMotor;
	rev::spark::SparkMax* mTurningMotor;

	rev::spark::SparkClosedLoopController* mDrivingClosedLoopController;
	rev::spark::SparkClosedLoopController* mTurningClosedLoopController;

	rev::spark::SparkRelativeEncoder* mDrivingEncoder;
	rev::spark::SparkAbsoluteEncoder* mTurningAbsoluteEncoder;

	// For simulation
	bool mRobotIsSimulated;
	frc::DCMotor* mDrivingGearBox;
	frc::DCMotor* mTurningGearBox;
	rev::spark::SparkMaxSim* mDrivingMotorSim;
	rev::spark::SparkMaxSim* mTurningMotorSim;

	frc::Rotation2d mTurningCurrentAngle;

	frc::SwerveModuleState mOptimizedState;

	frc::SwerveModuleState mModuleState;
	frc::SwerveModulePosition mModulePosition;
};
