// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

#include "Constants.h"

SubShooter::SubShooter()
{
	mLeaderShooterController = new rev::spark::SparkMax{CANid::kLeaderMotorShooterID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mFollowerShooterController = new rev::spark::SparkMax{CANid::kFollowerMotorShooterID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mRelativeEncoder = new rev::spark::SparkRelativeEncoder{mLeaderShooterController->GetEncoder()};
	mFeedforward = new frc::SimpleMotorFeedforward<units::turns>{ShooterConstants::kS, ShooterConstants::kV};
	mSparkConfigLeaderShooter = new rev::spark::SparkMaxConfig{};
	mSparkConfigFollowerShooter = new rev::spark::SparkMaxConfig{};
	Configure();
}

// This method will be called once per scheduler run
void SubShooter::Periodic()
{
	frc::SmartDashboard::PutNumber("shooter/velocity", getVelocity().value());
}

void SubShooter::setVoltage(units::volt_t iVoltage)
{
	mLeaderShooterController->SetVoltage(iVoltage);
};

void SubShooter::setVelocity(units::turns_per_second_t iNextVelocity)
{
	mLeaderShooterController->SetVoltage(mFeedforward->Calculate(iNextVelocity));
};

units::turns_per_second_t SubShooter::getVelocity()
{
	return units::revolutions_per_minute_t(mRelativeEncoder->GetVelocity());
};

std::array<rev::REVLibError, 2> SubShooter::Configure()
{
	mSparkConfigLeaderShooter->Inverted(ShooterConstants::kInverted);
	mSparkConfigLeaderShooter->SetIdleMode(ShooterConstants::kIdleMode);

	std::array<rev::REVLibError, 2> oConfigureResult;
	oConfigureResult[0] = mLeaderShooterController->Configure(*mSparkConfigLeaderShooter, ShooterConstants::kReset, ShooterConstants::kPersist);
	mSparkConfigFollowerShooter->Apply(*mSparkConfigLeaderShooter).Follow(CANid::kLeaderMotorShooterID, ShooterConstants::kFollowerinverted);
	oConfigureResult[1] = mFollowerShooterController->Configure(*mSparkConfigFollowerShooter, ShooterConstants::kReset, ShooterConstants::kPersist);
	return oConfigureResult;
};
