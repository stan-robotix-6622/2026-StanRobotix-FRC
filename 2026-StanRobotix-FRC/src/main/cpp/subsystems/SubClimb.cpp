// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimb.h"

SubClimb::SubClimb() {
    mSparkMax1 = new rev::spark::SparkMax(ClimbConstants::deviceIDSparkMax1, ClimbConstants::motorTypeSparkMax1);
    mSparkMax2 = new rev::spark::SparkMax(ClimbConstants::deviceIDSparkMax2, ClimbConstants::motorTypeSparkMax2);

    mSparkMaxConfig1 = new rev::spark::SparkMaxConfig;
    mSparkMaxConfig2 = new rev::spark::SparkMaxConfig;

    mSparkMaxConfig1->Inverted(true);
    mSparkMaxConfig2->Inverted(false);

    mSparkMax1->Configure(*mSparkMaxConfig1, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    mSparkMax2->Configure(*mSparkMaxConfig2, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    mSparkRelativeEncoder1 = new rev::spark::SparkRelativeEncoder{mSparkMax1->GetEncoder()};
    mSparkRelativeEncoder2 = new rev::spark::SparkRelativeEncoder{mSparkMax2->GetEncoder()};
}

// This method will be called once per scheduler run;
void SubClimb::Periodic() {}

void SubClimb::SetSpeed(double iSpeed) {
    mSparkMax1->Set(iSpeed);
    mSparkMax2->Set(iSpeed);
}

void SubClimb::StopMotor() {
    mSparkMax1->StopMotor();
    mSparkMax2->StopMotor();
}

double SubClimb::GetPosition() {
   // return mSparkRelativeEncoder1->GetPosition();
   return (mSparkRelativeEncoder1->GetPosition() + mSparkRelativeEncoder2->GetPosition()) / 2;
}

void SubClimb::ResetPosition() {
    mSparkRelativeEncoder1->SetPosition(0);
    mSparkRelativeEncoder2->SetPosition(0);
}
