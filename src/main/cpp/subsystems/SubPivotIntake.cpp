// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivotIntake.h"

#include <frc/RobotBase.h>
#include <frc/Alert.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>

#include <numbers>

#include "Configs.h"
#include "RobotixLib.hpp"

SubPivotIntake::SubPivotIntake()
{
	mLeftPivotMotor = new rev::spark::SparkMax{CANid::kLeaderMotorPivotID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mRightPivotMotor = new rev::spark::SparkMax{CANid::kFollowerMotorPivotID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mLeftEncoder = new rev::spark::SparkRelativeEncoder{mLeftPivotMotor->GetEncoder()};
	mRightEncoder = new rev::spark::SparkRelativeEncoder{mRightPivotMotor->GetEncoder()};
	mLeftFeedForward = new frc::ArmFeedforward{PivotConstants::Left::kS, PivotConstants::Left::kG, PivotConstants::Left::kV};
	mRightFeedForward = new frc::ArmFeedforward{PivotConstants::Right::kS, PivotConstants::Right::kG, PivotConstants::Right::kV};

	Configure();

	// Simulation
	if (frc::RobotBase::IsSimulation()) {
		mLeftGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
		mRightGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
		mLeftMotorSim = new rev::spark::SparkMaxSim{mLeftPivotMotor, mLeftGearBox};
		mRightMotorSim = new rev::spark::SparkMaxSim{mRightPivotMotor, mRightGearBox};
		mLeftArmPlant = new frc::LinearSystem<2, 1, 2>{frc::LinearSystemId::SingleJointedArmSystem(*mLeftGearBox, kLeftMOI, PivotConstants::kGearRatio)};
		mRightArmPlant = new frc::LinearSystem<2, 1, 2>{frc::LinearSystemId::SingleJointedArmSystem(*mRightGearBox, kRightMOI, PivotConstants::kGearRatio)};
		mLeftArmSim = new frc::sim::SingleJointedArmSim{*mLeftArmPlant, *mLeftGearBox, PivotConstants::kGearRatio, 15_in, 0_deg, 66_deg, true, units::radian_t(PivotConstants::Left::kOffset), {0.0, 0.0}};
		mRightArmSim = new frc::sim::SingleJointedArmSim{*mRightArmPlant, *mRightGearBox, PivotConstants::kGearRatio, 15_in, 0_deg, 66_deg, true, units::radian_t(PivotConstants::Right::kOffset), {0.0, 0.0}};
	}
}

void SubPivotIntake::Periodic() {}

void SubPivotIntake::Stop()
{
	mLeftPivotMotor->StopMotor();
	mRightPivotMotor->StopMotor();
}

void SubPivotIntake::SetLeftVoltage(units::volt_t iVoltage)
{
	mLeftPivotMotor->SetVoltage(iVoltage);

	if (frc::RobotBase::IsSimulation()) {
		mLeftArmSim->SetInputVoltage(iVoltage);
		mLeftArmSim->Update(0.02_s);
		mLeftMotorSim->iterate(mLeftArmSim->GetVelocity().value(), 12, 0.02);
	}
}

void SubPivotIntake::SetRightVoltage(units::volt_t iVoltage)
{
	mRightPivotMotor->SetVoltage(iVoltage);

	if (frc::RobotBase::IsSimulation()) {
		mRightArmSim->SetInputVoltage(iVoltage);
		mRightArmSim->Update(0.02_s);
		mRightMotorSim->iterate(mRightArmSim->GetVelocity().value(), 12, 0.02);
	}
}

void SubPivotIntake::SetLeftVelocity(units::radians_per_second_t iVelocity)
{
	mLeftPivotMotor->SetVoltage(mLeftFeedForward->Calculate(GetLeftAngle(), iVelocity));

	if (frc::RobotBase::IsSimulation()) {
		mLeftArmSim->SetInputVoltage(mLeftFeedForward->Calculate(GetLeftAngle(), iVelocity));
		mLeftArmSim->Update(0.02_s);
		mLeftMotorSim->iterate(mLeftArmSim->GetVelocity().value(), 12, 0.02);
	}
}

void SubPivotIntake::SetRightVelocity(units::radians_per_second_t iVelocity)
{
	mRightPivotMotor->SetVoltage(mRightFeedForward->Calculate(GetRightAngle(), iVelocity));

	if (frc::RobotBase::IsSimulation()) {
		mRightArmSim->SetInputVoltage(mRightFeedForward->Calculate(GetRightAngle(), iVelocity));
		mRightArmSim->Update(0.02_s);
		mRightMotorSim->iterate(mRightArmSim->GetVelocity().value(), 12, 0.02);
	}
}

void SubPivotIntake::KeepPosition()
{
	mLeftPivotMotor->SetVoltage(mLeftFeedForward->Calculate(GetLeftAngle(), 0_rad_per_s));
	mRightPivotMotor->SetVoltage(mRightFeedForward->Calculate(GetRightAngle(), 0_rad_per_s));
}

units::radian_t SubPivotIntake::GetLeftAngle()
{
	return units::radian_t(PivotConstants::Left::kOffset + mLeftEncoder->GetPosition());
}

units::radian_t SubPivotIntake::GetRightAngle()
{
	return units::radian_t(PivotConstants::Right::kOffset + mRightEncoder->GetPosition());
}

std::array<frc::Alert*, 2> SubPivotIntake::Configure()
{
	return {
		robotixLib::getAlertForREVErrorMessage(
			mLeftPivotMotor->Configure(Configs::Pivot::LeftConfig(), IntakeConstants::kReset, IntakeConstants::kPersist),
			"Left Pivot"),
		robotixLib::getAlertForREVErrorMessage(
			mRightPivotMotor->Configure(Configs::Pivot::RightConfig(), IntakeConstants::kReset, IntakeConstants::kPersist),
			"Right Pivot")};
};

void SubPivotIntake::InitSendable(wpi::SendableBuilder& builder)
{
	builder.SetSmartDashboardType("pivot");
	builder.AddDoubleProperty("position left", [this] { return mLeftEncoder->GetPosition(); }, nullptr);
	builder.AddDoubleProperty("position right", [this] { return mLeftEncoder->GetPosition(); }, nullptr);
	builder.AddDoubleProperty("angle left (radians)", [this] { return GetLeftAngle().value(); }, nullptr);
	builder.AddDoubleProperty("angle right (radians)", [this] { return GetRightAngle().value(); }, nullptr);
	builder.AddDoubleProperty("angle left (degrees)", [this] { return units::degree_t(GetLeftAngle()).value(); }, nullptr);
	builder.AddDoubleProperty("angle right (degrees)", [this] { return units::degree_t(GetRightAngle()).value(); }, nullptr);
	builder.AddDoubleProperty("kV left", [this] { return mLeftFeedForward->GetKv().value(); }, [this](double iKv) { return mLeftFeedForward->SetKv(robotixLib::templateUnits::VoltageInverse<units::radians_per_second>(iKv)); });
	builder.AddDoubleProperty("kV left", [this] { return mLeftFeedForward->GetKv().value(); }, [this](double iKv) { return mLeftFeedForward->SetKv(robotixLib::templateUnits::VoltageInverse<units::radians_per_second>(iKv)); });
	builder.AddDoubleProperty("kG left", [this] { return mLeftFeedForward->GetKg().value(); }, [this](double iKg) { return mLeftFeedForward->SetKg(units::volt_t(iKg)); });
	builder.AddDoubleProperty("kG right", [this] { return mRightFeedForward->GetKg().value(); }, [this](double iKg) { return mRightFeedForward->SetKg(units::volt_t(iKg)); });
	builder.AddDoubleProperty("kS right", [this] { return mRightFeedForward->GetKs().value(); }, [this](double iKs) { return mRightFeedForward->SetKs(units::volt_t(iKs)); });
	builder.AddDoubleProperty("kS right", [this] { return mRightFeedForward->GetKs().value(); }, [this](double iKs) { return mRightFeedForward->SetKs(units::volt_t(iKs)); });
}
