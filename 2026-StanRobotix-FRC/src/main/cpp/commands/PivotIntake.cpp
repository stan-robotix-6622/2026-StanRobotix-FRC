// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntake.h"

#include "Constants.h"

PivotIntake::PivotIntake(SubPivotIntake* iPivotIntake, StatePivotIntake iTarget)
{
	mPivotIntake = iPivotIntake;
	mPIDController = new frc::PIDController{PivotConstants::kP, PivotConstants::kI, PivotConstants::kD};
	mState = iTarget;
	frc::SmartDashboard::PutData("pivot/Arm PID", mPIDController);
	AddRequirements(mPivotIntake);
	// Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PivotIntake::Initialize()
{
	mPIDController->Reset();
	switch (mState)
	{
		case kUp:
			mPIDController->SetSetpoint(PivotConstants::setpointUp);
			break;

		case kDown:
			mPIDController->SetSetpoint(PivotConstants::setpointDown);
			break;
	}
}

// Called repeatedly when this Command is scheduled to run
void PivotIntake::Execute()
{
	double wVoltage = mPIDController->Calculate(mPivotIntake->GetAngle());
	frc::SmartDashboard::PutNumber("pivot/Arm PID adjust", wVoltage);
	mPivotIntake->SetVoltage(wVoltage + (PivotConstants::kG.value() * cos(mPivotIntake->GetAngle())));
}

// Called once the command ends or is interrupted.
void PivotIntake::End(bool interrupted) {}

// Returns true when the command should end.
bool PivotIntake::IsFinished()
{
	return false;
}
