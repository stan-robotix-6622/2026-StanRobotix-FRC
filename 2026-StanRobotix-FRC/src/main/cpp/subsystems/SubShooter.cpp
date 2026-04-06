// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Configs.h"

SubShooter::SubShooter()
{
	mLeaderShooterController = new rev::spark::SparkMax{CANid::kLeaderMotorShooterID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mFollowerShooterController = new rev::spark::SparkMax{CANid::kFollowerMotorShooterID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mRelativeEncoder = new rev::spark::SparkRelativeEncoder{mLeaderShooterController->GetEncoder()};
	mFeedforward = new frc::SimpleMotorFeedforward<units::turns>{ShooterConstants::kS, ShooterConstants::kV};

	mClossedLoopController = new rev::spark::SparkClosedLoopController{mLeaderShooterController->GetClosedLoopController()};

	Configure();

	// Simulation
	if (frc::RobotBase::IsSimulation()) {
		mRobotIsSimulated = true;
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
};

void SubShooter::setDesiredVelocity(units::turns_per_second_t iNextVelocity)
{
	mDesiredVelocity = iNextVelocity;
	mClossedLoopController->SetSetpoint(iNextVelocity.value(), ShooterConstants::kShooterClosedLoopControlType);

	if (mRobotIsSimulated) {
		mFlywheelSim->SetInputVoltage(mFeedforward->Calculate(iNextVelocity));
		mFlywheelSim->Update(0.02_s);
		mLeaderMotorSim->iterate(units::turns_per_second_t(mFlywheelSim->GetAngularVelocity()).value(), 12, 0.02);
		mFollowMotorSim->iterate(units::turns_per_second_t(mFlywheelSim->GetAngularVelocity()).value(), 12, 0.02);
	}
};

units::turns_per_second_t SubShooter::getVelocity()
{
	return units::turns_per_second_t(mRelativeEncoder->GetVelocity());
};

std::array<rev::REVLibError, 2> SubShooter::Configure()
{
	return {
		mLeaderShooterController->Configure(Configs::Shooter::LeaderConfig(), ShooterConstants::kReset, ShooterConstants::kPersist),
		mFollowerShooterController->Configure(Configs::Shooter::FollowerConfig(), ShooterConstants::kReset, ShooterConstants::kPersist)};
};

void SubShooter::InitSendable(wpi::SendableBuilder& builder)
{
	builder.SetSmartDashboardType("shooter");
	builder.AddDoubleProperty("velocity (tps)", [this] { return getVelocity().value(); }, nullptr);
	builder.AddDoubleProperty("velocity (rpm)", [this] { return units::revolutions_per_minute_t(getVelocity()).value(); }, nullptr);
	builder.AddDoubleProperty("kA", [this] { return mFeedforward->GetKa().value(); }, [this](double iKa) { return mFeedforward->SetKa(TemplateUnits::VoltageInverse<units::turns_per_second_squared>(iKa)); });
	builder.AddDoubleProperty("kV", [this] { return mFeedforward->GetKv().value(); }, [this](double iKv) { return mFeedforward->SetKv(TemplateUnits::VoltageInverse<units::turns_per_second>(iKv)); });
	builder.AddDoubleProperty("kS", [this] { return mFeedforward->GetKs().value(); }, [this](double iKs) { return mFeedforward->SetKs(units::volt_t(iKs)); });
}

bool SubShooter::atDesiredVelocity()
{
	return units::math::abs(getVelocity() - mDesiredVelocity) < 0.5_tps && mDesiredVelocity != 0_tps;
}
