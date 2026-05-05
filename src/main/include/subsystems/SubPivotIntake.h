// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <wpi/sendable/SendableBuilder.h>

class SubPivotIntake : public frc2::SubsystemBase {
 public:
	SubPivotIntake();

	void Stop();

	void KeepPosition();

	void SetVoltage(units::volt_t iVoltage);

	void SetVelocity(units::radians_per_second_t iVelocity);

	units::radian_t GetAngle();

	void Periodic() override;

	void InitSendable(wpi::SendableBuilder& builder) override;

 private:
	rev::spark::SparkMax* mPivotMotor;
	rev::spark::SparkRelativeEncoder* mEncoder;
	frc::ArmFeedforward* mFeedForward;

	// For simulation
	bool mRobotIsSimulated = false;
	units::kilogram_square_meter_t kMOI = (2_lb * 12_in * 12_in) / 3;
	frc::DCMotor* mGearBox;
	rev::spark::SparkMaxSim* mMotorSim;
	frc::sim::SingleJointedArmSim* mArmSim;
	frc::LinearSystem<2, 1, 2>* mArmPlant;
};
