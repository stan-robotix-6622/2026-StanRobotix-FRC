// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivotIntake.h"

#include <numbers>

#include "Constants.h"

SubPivotIntake::SubPivotIntake()
{
	mPivotMotor = new rev::spark::SparkMax{CANid::kMotorPivotID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mEncoder = new rev::spark::SparkRelativeEncoder{mPivotMotor->GetEncoder()};
	mFeedForward = new frc::ArmFeedforward{0_V, PivotConstants::kG, 1_V / 1_rad_per_s};
	frc::SmartDashboard::PutNumber("pivot/Arm kG", PivotConstants::kG.value());

	mPivotMotorConfig = new rev::spark::SparkMaxConfig{};
	mPivotMotorConfig->Inverted(PivotConstants::kInverted);
	mPivotMotorConfig->SetIdleMode(PivotConstants::kIdleMode);
	mPivotMotor->Configure(*mPivotMotorConfig, PivotConstants::kReset, PivotConstants::kPersist);
}

// This method will be called once per scheduler run
void SubPivotIntake::Periodic()
{
	frc::SmartDashboard::PutNumber("pivot/Arm Position", mEncoder->GetPosition());
	frc::SmartDashboard::PutNumber("pivot/Arm Angle", GetAngle());
}

void SubPivotIntake::Stop()
{
	mPivotMotor->StopMotor();
}

void SubPivotIntake::SetVoltage(double iVoltage)
{
	mPivotMotor->SetVoltage(units::volt_t(iVoltage));
}

void SubPivotIntake::KeepPosition()
{
	// units::volt_t wVoltage = PivotConstants::kG * cos(GetAngle());
	units::volt_t wVoltage = units::volt_t(frc::SmartDashboard::GetNumber("pivot/Arm kG", PivotConstants::kG.value()) * cos(GetAngle()));
	frc::SmartDashboard::PutNumber("pivot/kG voltage", wVoltage.value());
	mPivotMotor->SetVoltage(wVoltage);
}

double SubPivotIntake::GetAngle()
{
	return (PivotConstants::kOffset + mEncoder->GetPosition()) * 2 * std::numbers::pi / PivotConstants::kGearRatio;
}
