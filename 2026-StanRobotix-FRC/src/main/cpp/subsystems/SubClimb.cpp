// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimb.h"
#include "Constants.h"

#include <frc/StateSpaceUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

SubClimb::SubClimb() {
    mSparkMax1 = new rev::spark::SparkMax(ClimbConstants::kDeviceIDSparkMax1, ClimbConstants::kMotorTypeSparkMax1);
    mSparkMax2 = new rev::spark::SparkMax(ClimbConstants::kDeviceIDSparkMax2, ClimbConstants::kMotorTypeSparkMax2);

    mSparkMaxConfig1 = new rev::spark::SparkMaxConfig;
    mSparkMaxConfig2 = new rev::spark::SparkMaxConfig;

    mSparkMaxConfig1->Inverted(true);
    mSparkMaxConfig2->Inverted(false);

    mSparkMax1->Configure(*mSparkMaxConfig1, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);
    mSparkMax2->Configure(*mSparkMaxConfig2, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);

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
   return (abs(mSparkRelativeEncoder1->GetPosition()) + abs(mSparkRelativeEncoder2->GetPosition())) / 2;
}

frc2::CommandPtr SubClimb::GetClimbCommand(ClimbCommands iCommand) {
    switch (iCommand) {
        case ClimbUp:
            return;
        case ClimbDown:
            return;
    }
}
