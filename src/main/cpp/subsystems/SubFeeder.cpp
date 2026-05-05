// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "Configs.h"

SubFeeder::SubFeeder()
{
	mFeederController = new rev::spark::SparkMax{CANid::kMotorFeederID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	Configure();

	// Simulation
	if (frc::RobotBase::IsSimulation()) {
		mRobotIsSimulated = true;
		mGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
		mMotorSim = new rev::spark::SparkMaxSim{mFeederController, mGearBox};
	}
}

void SubFeeder::Periodic()
{
	frc::SmartDashboard::PutBoolean("Dashboard/isFeederOn", isFeederOn());
}

void SubFeeder::setVoltage(units::volt_t iOutput)
{
	mFeederController->SetVoltage(iOutput);

	if (mRobotIsSimulated) {
		mMotorSim->SetAppliedOutput(iOutput / 12_V);
	}
};

rev::REVLibError SubFeeder::Configure()
{
	return mFeederController->Configure(Configs::Feeder::Config(), FeederConstants::kReset, FeederConstants::kPersist);
};

frc2::CommandPtr SubFeeder::getFeedShooterCommand(units::volt_t iVoltage)
{
	return frc2::cmd::RunEnd(
			[this, iVoltage] {
				setVoltage(iVoltage);
			},

			[this] {
				setVoltage(0_V);
			},
			{});
}

bool SubFeeder::isFeederOn()
{
	return mFeederController->GetAppliedOutput() != 0;
}
