// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimb.h"

SubClimb::SubClimb() {
    mSparkMax1 = new rev::spark::SparkMax(ClimbConstants::deviceIDSparkMax1, ClimbConstants::motorTypeSparkMax1);
    mSparkMax2 = new rev::spark::SparkMax(ClimbConstants::deviceIDSparkMax2, ClimbConstants::motorTypeSparkMax2);
    mSparkMax1->SetInverted(true);
}


// This method will be called once per scheduler run
void SubClimb::Periodic() {}

void SubClimb::SetSpeed(double iSpeed) {
    mSparkMax1->Set(iSpeed);
    mSparkMax2->Set(iSpeed);
}