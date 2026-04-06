// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FullIntake.h"

#include <frc2/command/Commands.h>

frc2::CommandPtr FullIntake::FullIntakeCommand(SubIntake* iIntake, SubPivotIntake* iPivot, PivotIntake::StatePivotIntake itargetState)
{
	if (itargetState == PivotIntake::StatePivotIntake::kDown) {
		return PivotIntake(iPivot, PivotIntake::StatePivotIntake::kDown).AlongWith(iIntake->getIntakeCommand());
	}
	else if (itargetState == PivotIntake::StatePivotIntake::kUp) {
		return PivotIntake(iPivot, PivotIntake::StatePivotIntake::kUp).AlongWith(iIntake->Idle());
	}
	else if (itargetState == PivotIntake::StatePivotIntake::kIn) {
		return PivotIntake(iPivot, PivotIntake::StatePivotIntake::kIn).AlongWith(iIntake->Idle());
	}
	return frc2::cmd::None();
}
