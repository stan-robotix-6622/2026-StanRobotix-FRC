// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntake.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

PivotIntake::PivotIntake(SubPivotIntake* iPivotIntake, StatePivotIntake iTarget)
{
	mPivotIntake = iPivotIntake;
	AddRequirements(mPivotIntake);

	mState = iTarget;

	mLeftPIDController = new frc::PIDController{PivotConstants::Left::kP, PivotConstants::Left::kI, PivotConstants::Left::kD};
	mRightPIDController = new frc::PIDController{PivotConstants::Right::kP, PivotConstants::Right::kI, PivotConstants::Right::kD};
// 	frc::SmartDashboard::PutData("commands/pivot intake/Arm Left PID", mLeftPIDController);
// 	frc::SmartDashboard::PutData("commands/pivot intake/Arm Right PID", mRightPIDController);
}

// Called when the command is initially scheduled.
void PivotIntake::Initialize()
{
	mLeftPIDController->Reset();
	mRightPIDController->Reset();
	switch (mState) {
		case kUp:
			mLeftPIDController->SetSetpoint(PivotConstants::kSetpointUp);
			mRightPIDController->SetSetpoint(PivotConstants::kSetpointUp);
			break;
		case kDown:
			mLeftPIDController->SetSetpoint(PivotConstants::kSetpointDown);
			mRightPIDController->SetSetpoint(PivotConstants::kSetpointDown);
			break;
		case kIn:
		mLeftPIDController->SetSetpoint(PivotConstants::kSetpointIn);
		mRightPIDController->SetSetpoint(PivotConstants::kSetpointIn);
			break;
	}
	if (frc::SmartDashboard::GetNumber("tunable/Pivot Left kP", PivotConstants::Left::kP) != mLeftPIDController->GetP())
	{
		mLeftPIDController->SetP(frc::SmartDashboard::GetNumber("tunable/Pivot Left kP", PivotConstants::Left::kP));
		frc::DataLogManager::Log("Left kP was changed to : " + std::to_string(mLeftPIDController->GetP()));
	}
	if (frc::SmartDashboard::GetNumber("tunable/Pivot Right kP", PivotConstants::Right::kP) != mRightPIDController->GetP())
	{
		mRightPIDController->SetP(frc::SmartDashboard::GetNumber("tunable/Pivot Right kP", PivotConstants::Right::kP));
		frc::DataLogManager::Log("Right kP was changed to : " + std::to_string(mRightPIDController->GetP()));
	}
}

// Called repeatedly when this Command is scheduled to run
void PivotIntake::Execute()
{
	frc::SmartDashboard::PutNumber("commands/pivot intake/Left PID Setpoint", mLeftPIDController->GetSetpoint());
	frc::SmartDashboard::PutNumber("commands/pivot intake/Right PID Setpoint", mRightPIDController->GetSetpoint());
	frc::SmartDashboard::PutNumber("commands/pivot intake/Left PID Angle", mPivotIntake->GetLeftAngle().value());
	units::volt_t wLeftVoltage = units::volt_t(mLeftPIDController->Calculate(mPivotIntake->GetLeftAngle().value()));
	units::volt_t wRightVoltage = units::volt_t(mRightPIDController->Calculate(mPivotIntake->GetRightAngle().value()));
	frc::SmartDashboard::PutNumber("commands/pivot intake/Left PID Calculate", wLeftVoltage.value());
	frc::SmartDashboard::PutNumber("commands/pivot intake/Right PID Calculate", wRightVoltage.value());
	frc::SmartDashboard::PutNumber("commands/pivot intake/Left PID Voltage", wLeftVoltage.value());
	frc::SmartDashboard::PutNumber("commands/pivot intake/Right PID Voltage", wRightVoltage.value());
	mPivotIntake->SetLeftVoltage(wLeftVoltage + (PivotConstants::Left::kG * cos(mPivotIntake->GetLeftAngle().value())));
	mPivotIntake->SetRightVoltage(wRightVoltage + (PivotConstants::Right::kG * cos(mPivotIntake->GetRightAngle().value())));
}

// Called once the command ends or is interrupted.
void PivotIntake::End(bool interrupted)
{
	mPivotIntake->Stop();
}

// Returns true when the command should end.
bool PivotIntake::IsFinished()
{
	return false;
}
