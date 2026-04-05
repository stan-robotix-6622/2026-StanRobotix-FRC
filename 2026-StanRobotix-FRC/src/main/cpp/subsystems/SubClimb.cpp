// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimb.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

#include "commands/Climb.h"

SubClimb::SubClimb()
{
	mSparkMaxLeader = new rev::spark::SparkMax(CANid::kMotorClimbLeaderID, ClimbConstants::kMotorTypeLeader);
	mSparkMaxFollower = new rev::spark::SparkMax(CANid::kMotorClimbFollowerID, ClimbConstants::kMotorTypeFollower);

	mSparkMaxConfigLeader = new rev::spark::SparkMaxConfig;
	mSparkMaxConfigFollower = new rev::spark::SparkMaxConfig;

	mSparkMaxConfigLeader->Inverted(ClimbConstants::kInverted);
	mSparkMaxConfigLeader->SetIdleMode(ClimbConstants::kIdleMode);

	mSparkMaxConfigFollower->Apply(*mSparkMaxConfigLeader);
	mSparkMaxConfigFollower->Follow(CANid::kMotorClimbLeaderID, ClimbConstants::kInverseFollowerMotor);

	mSparkMaxLeader->Configure(*mSparkMaxConfigLeader, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
	mSparkMaxFollower->Configure(*mSparkMaxConfigFollower, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);

	mSparkRelativeEncoder = new rev::spark::SparkRelativeEncoder{mSparkMaxLeader->GetEncoder()};

	mHighPassFilter = new frc::LinearFilter<units::ampere_t>{frc::LinearFilter<units::ampere_t>::HighPass(0.1, 0.02_s)};
}

// This method will be called once per scheduler run;
void SubClimb::Periodic()
{
	mCurrent = GetCurrent();
	mCurrentFiltered = mHighPassFilter->Calculate(mCurrent).value();

	frc::SmartDashboard::PutNumber("climb/current", mCurrent.value());
	frc::SmartDashboard::PutNumber("climb/current filter", mCurrentFiltered);
}

void SubClimb::SetSpeed(double iSpeed)
{
	mSparkMaxLeader->Set(iSpeed);
}

void SubClimb::StopMotor()
{
	mSparkMaxLeader->StopMotor();
}

double SubClimb::GetPosition()
{
	return mSparkRelativeEncoder->GetPosition();
}

frc2::CommandPtr SubClimb::GetClimbCommand(Direction iDirection)
{
	return Climb(this, iDirection).ToPtr();
}

units::ampere_t SubClimb::GetCurrent()
{
	return units::ampere_t(mSparkMaxLeader->GetPeriodicStatus0().current);
}

double SubClimb::GetCurrentFiltered()
{
	return mCurrentFiltered;
}

void SubClimb::SetDownPosition(double iDownPosition)
{
	mDownPosition = iDownPosition;
}

double SubClimb::GetDownPosition()
{
	return mDownPosition;
}
