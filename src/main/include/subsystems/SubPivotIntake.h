// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Alert.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <wpi/sendable/SendableBuilder.h>

class SubPivotIntake : public frc2::SubsystemBase {
 public:
	SubPivotIntake();

	void Stop();

	void KeepPosition();

	void SetLeftVoltage(units::volt_t iVoltage);
	void SetRightVoltage(units::volt_t iVoltage);

	void SetLeftVelocity(units::radians_per_second_t iVelocity);
	void SetRightVelocity(units::radians_per_second_t iVelocity);

	units::radian_t GetLeftAngle();
	units::radian_t GetRightAngle();

	void Periodic() override;

	void InitSendable(wpi::SendableBuilder& builder) override;

	std::array<frc::Alert*, 2> Configure();

	frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

 private:
	rev::spark::SparkMax* mLeftPivotMotor;
	rev::spark::SparkMax* mRightPivotMotor;
	rev::spark::SparkRelativeEncoder* mLeftEncoder;
	rev::spark::SparkRelativeEncoder* mRightEncoder;
	frc::ArmFeedforward* mLeftFeedForward;
	frc::ArmFeedforward* mRightFeedForward;

	// For simulation
	units::kilogram_square_meter_t kLeftMOI = (1.5_lb * 15_in * 15_in) / 3;
	units::kilogram_square_meter_t kRightMOI = (1.5_lb * 15_in * 15_in) / 3;
	frc::DCMotor* mLeftGearBox;
	frc::DCMotor* mRightGearBox;
	rev::spark::SparkMaxSim* mLeftMotorSim;
	rev::spark::SparkMaxSim* mRightMotorSim;
	frc::sim::SingleJointedArmSim* mLeftArmSim;
	frc::sim::SingleJointedArmSim* mRightArmSim;
	frc::LinearSystem<2, 1, 2>* mLeftArmPlant;
	frc::LinearSystem<2, 1, 2>* mRightArmPlant;

	frc2::sysid::SysIdRoutine* mPivotRoutine;
	frc2::sysid::Config* mConfig;
	frc2::sysid::Mechanism* mMechanism;
};
