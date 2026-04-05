// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntake.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

PivotIntake::PivotIntake(SubPivotIntake* iPivotIntake, StatePivotIntake iTarget)
{
	mPivotIntake = iPivotIntake;
	AddRequirements(mPivotIntake);

	mState = iTarget;

	mPIDController = new frc::PIDController{PivotConstants::kP, PivotConstants::kI, PivotConstants::kD};
	frc::SmartDashboard::PutData("pivot/Arm PID", mPIDController);
}

// Called when the command is initially scheduled.
void PivotIntake::Initialize()
{
	mPIDController->Reset();
	switch (mState) {
		case kUp:
			mPIDController->SetSetpoint(PivotConstants::setpointUp);
			break;

		case kDown:
			mPIDController->SetSetpoint(PivotConstants::setpointDown);
			break;
		case kIn:
			mPIDController->SetSetpoint(PivotConstants::setpointIn);
			break;
	}
}

// Called repeatedly when this Command is scheduled to run
void PivotIntake::Execute()
{
	units::volt_t wVoltage = (units::volt_t)mPIDController->Calculate(mPivotIntake->GetAngle().value());
	// frc::SmartDashboard::PutNumber("pivot/Arm PID adjust", wVoltage.value());
	mPivotIntake->SetVoltage(wVoltage + (PivotConstants::kG * cos(mPivotIntake->GetAngle().value())));
}

// Called once the command ends or is interrupted.
void PivotIntake::End(bool interrupted) {}

// Returns true when the command should end.
bool PivotIntake::IsFinished()
{
	return false;
}
