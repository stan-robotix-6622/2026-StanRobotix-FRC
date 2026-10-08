// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Configs.h"
#include "RobotixLib.hpp"

SubShooter::SubShooter()
{
	mLeaderShooterController = new rev::spark::SparkMax{CANid::kLeaderMotorShooterID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mFollowerShooterController = new rev::spark::SparkMax{CANid::kFollowerMotorShooterID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mRelativeEncoder = new rev::spark::SparkRelativeEncoder{mLeaderShooterController->GetEncoder()};
	mFeedforward = new frc::SimpleMotorFeedforward<units::turns>{ShooterConstants::kS, ShooterConstants::kV};
	// mRoutine = new frc2::sysid::SysIdRoutine{
	// 		frc2::sysid::Config
	// 		{
	// 			ShooterConstants::SystemId::kRampRate,
	// 			ShooterConstants::SystemId::kStepVoltage,
	// 			ShooterConstants::SystemId::kTimeout
	// 		}
	// 	};
	mClossedLoopController = new rev::spark::SparkClosedLoopController{mLeaderShooterController->GetClosedLoopController()};

	Configure();

	// Simulation
	if (frc::RobotBase::IsSimulation()) {
		mLeaderGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
		mFollowGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
		mLeaderMotorSim = new rev::spark::SparkMaxSim{mLeaderShooterController, mLeaderGearBox};
		mFollowMotorSim = new rev::spark::SparkMaxSim{mFollowerShooterController, mFollowGearBox};
		mFlywheelPlant = new frc::LinearSystem<1, 1, 1>{frc::LinearSystemId::FlywheelSystem(frc::DCMotor::NEO(2), kMOI, ShooterConstants::kGearRatio)};
		mFlywheelSim = new frc::sim::FlywheelSim{*mFlywheelPlant, frc::DCMotor::NEO(2), {0.0}};
	}
}

void SubShooter::Periodic()
{
	frc::SmartDashboard::PutBoolean("Dashboard/atDesiredVelocity", atDesiredVelocity());
}

void SubShooter::setVoltage(units::volt_t iVoltage)
{
	mLeaderShooterController->SetVoltage(iVoltage);
}

void SubShooter::setVelocity(units::turns_per_second_t iNextVelocity)
{
	mClossedLoopController->SetSetpoint(iNextVelocity.value(), ShooterConstants::kShooterClosedLoopControlType);

	if (frc::RobotBase::IsSimulation()) {
		mFlywheelSim->SetInputVoltage(mFeedforward->Calculate(iNextVelocity));
		mFlywheelSim->Update(0.02_s);
		mLeaderMotorSim->iterate(units::turns_per_second_t(mFlywheelSim->GetAngularVelocity()).value(), 12, 0.02);
		mFollowMotorSim->iterate(units::turns_per_second_t(mFlywheelSim->GetAngularVelocity()).value(), 12, 0.02);
	}
}

void SubShooter::setTargetVelocity(units::turns_per_second_t iTargetVelocity)
{
	mTargetVelocity = iTargetVelocity;
}

units::turns_per_second_t SubShooter::getVelocity()
{
	return units::turns_per_second_t(mRelativeEncoder->GetVelocity());
}

std::array<frc::Alert*, 2> SubShooter::Configure()
{
	return {
		robotixLib::getAlertForREVErrorMessage(
			mLeaderShooterController->Configure(Configs::Shooter::LeaderConfig(), ShooterConstants::kReset, ShooterConstants::kPersist),
			"Shooter Leader"),
		robotixLib::getAlertForREVErrorMessage(
			mFollowerShooterController->Configure(Configs::Shooter::FollowerConfig(), ShooterConstants::kReset, ShooterConstants::kPersist),
			"Shooter Follower")};
};

void SubShooter::InitSendable(wpi::SendableBuilder& builder)
{
	builder.SetSmartDashboardType("shooter");
	builder.AddDoubleProperty("velocity (tps)", [this] { return getVelocity().value(); }, nullptr);
	builder.AddDoubleProperty("velocity (rpm)", [this] { return units::revolutions_per_minute_t(getVelocity()).value(); }, nullptr);
	builder.AddDoubleProperty("kA", [this] { return mFeedforward->GetKa().value(); }, [this](double iKa) { return mFeedforward->SetKa(robotixLib::templateUnits::VoltageInverse<units::turns_per_second_squared>(iKa)); });
	builder.AddDoubleProperty("kV", [this] { return mFeedforward->GetKv().value(); }, [this](double iKv) { return mFeedforward->SetKv(robotixLib::templateUnits::VoltageInverse<units::turns_per_second>(iKv)); });
	builder.AddDoubleProperty("kS", [this] { return mFeedforward->GetKs().value(); }, [this](double iKs) { return mFeedforward->SetKs(units::volt_t(iKs)); });
}

bool SubShooter::atDesiredVelocity()
{
	return units::math::abs(getVelocity() - mTargetVelocity) < units::turns_per_second_t(frc::SmartDashboard::GetNumber("tunable/Shooter tolerance", ShooterConstants::PIDConstants::kTolerance.value())) && mTargetVelocity != 0_tps;
}

// void SubShooter::SysIdRoutine(frc2::sysid::SysIdRoutine routine)
// {
// 	routine.
// }
