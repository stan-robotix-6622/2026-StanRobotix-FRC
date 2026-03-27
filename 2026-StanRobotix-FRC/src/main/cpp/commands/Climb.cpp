// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climb.h"

#include <frc/smartdashboard/SmartDashboard.h>

Climb::Climb(SubClimb * iSubClimb, SubClimb::Direction iDirection) {
  // Use addRequirements() here to declare subsystem dependencies.
  mSubClimb = iSubClimb;
  AddRequirements(mSubClimb);
  mPIDController = new frc::PIDController(0, 0, 0);

  frc::SmartDashboard::PutData("climb/PID Controller", mPIDController);
  mDirection = iDirection;
}

// Called when the command is initially scheduled.
void Climb::Initialize() {
  switch (mDirection) {
    case SubClimb::Up:
      mPIDController->SetPID(ClimbConstants::kUpP, ClimbConstants::kUpI, ClimbConstants::kUpD);
      mPIDController->SetSetpoint(ClimbConstants::kSetpointUp);
      break;
    case SubClimb::Down:
    mPIDController->SetPID(ClimbConstants::kDownP, ClimbConstants::kDownI, ClimbConstants::kDownD);
      mPIDController->SetSetpoint(ClimbConstants::kSetpointDown);
      break;
    case SubClimb::Lift:
      mPIDController->SetPID(ClimbConstants::kLiftP, ClimbConstants::kLiftI, ClimbConstants::kLiftD);
      mPIDController->SetSetpoint(ClimbConstants::kSetpointDown);
      break;
    default:
      break;
  }
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  mSubClimb->SetSpeed(mPIDController->Calculate(mSubClimb->GetPosition()));
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {
  mSubClimb->StopMotors();
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return false;
}
