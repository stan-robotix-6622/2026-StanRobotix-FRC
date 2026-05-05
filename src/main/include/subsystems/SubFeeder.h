// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkMax.h>

#include <units/voltage.h>

class SubFeeder : public frc2::SubsystemBase {
 public:
	SubFeeder();

	void setVoltage(units::volt_t iOutput);
	rev::REVLibError Configure();
	frc2::CommandPtr getFeedShooterCommand(units::volt_t iVoltage);
	bool isFeederOn();

	void Periodic() override;

 private:
	rev::spark::SparkMax* mFeederController;

	// For simulation
	bool mRobotIsSimulated = false;
	frc::DCMotor* mGearBox;
	rev::spark::SparkMaxSim* mMotorSim;
};
