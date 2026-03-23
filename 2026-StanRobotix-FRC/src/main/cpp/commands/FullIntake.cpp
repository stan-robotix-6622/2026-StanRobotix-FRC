// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FullIntake.h"

frc2::CommandPtr FullIntake::FullIntakeCommand(SubIntake* iIntake, SubPivotIntake* iPivot, PivotIntake::StatePivotIntake itargetState)
{
	// Use addRequirements() here to declare subsystem dependencies.
	if (itargetState == PivotIntake::StatePivotIntake::kDown)
	{
		return PivotIntake(iPivot, PivotIntake::StatePivotIntake::kDown).AlongWith(iIntake->getIntakeCommand());
	}
	else if (itargetState == PivotIntake::StatePivotIntake::kUp)
	{
		return PivotIntake(iPivot, PivotIntake::StatePivotIntake::kUp).AlongWith(iIntake->Idle());
	};
	return frc2::cmd::None();
}
