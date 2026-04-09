// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/FlywheelSim.h>
#include <rev/sim/SparkMaxSim.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>

class SubShooter : public frc2::SubsystemBase {
 public:
	SubShooter();

	void setVelocity(units::turns_per_second_t iNextVelocity);
	void setTargetVelocity(units::turns_per_second_t iTargetVelocity);
	void setVoltage(units::volt_t iVoltage);
	units::turns_per_second_t getVelocity();

	// The first index of the array is the result of the Leader's configuration and
	// the second is the result of the Follower's configuration
	std::array<rev::REVLibError, 2> Configure();

	void Periodic() override;

	void InitSendable(wpi::SendableBuilder& builder) override;

	bool atDesiredVelocity();

 private:
	frc::SimpleMotorFeedforward<units::turns>* mFeedforward;

	rev::spark::SparkMax* mLeaderShooterController;
	rev::spark::SparkMax* mFollowerShooterController;
	rev::spark::SparkRelativeEncoder* mRelativeEncoder;
	rev::spark::SparkClosedLoopController* mClossedLoopController;

	// For simulation
	bool mRobotIsSimulated = false;
	units::kilogram_square_meter_t kMOI = 2_in * 2_in * 1.56_lb + 1.625_in * 1.625_in * 1.2_lb;
	frc::DCMotor* mLeaderGearBox;
	frc::DCMotor* mFollowGearBox;
	rev::spark::SparkMaxSim* mLeaderMotorSim;
	rev::spark::SparkMaxSim* mFollowMotorSim;
	frc::sim::FlywheelSim* mFlywheelSim;
	frc::LinearSystem<1, 1, 1>* mFlywheelPlant;

	units::turns_per_second_t mTargetVelocity = 0_tps;
};
