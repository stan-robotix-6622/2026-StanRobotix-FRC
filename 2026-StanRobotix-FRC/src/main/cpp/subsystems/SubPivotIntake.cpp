// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivotIntake.h"


SubPivotIntake::SubPivotIntake() {
    mPivotMotor1 = new rev::spark::SparkMax(PivotConstants::kMotorPivotid1, rev::spark::SparkLowLevel::MotorType::kBrushless);
    mPivotMotor2 = new rev::spark::SparkMax(PivotConstants::kMotorPivotid2, rev::spark::SparkLowLevel::MotorType::kBrushless);
}



// This method will be called once per scheduler run
void SubPivotIntake::Periodic() {}


void SubPivotIntake::Stop() {
    mPivotMotor1->StopMotor();
    mPivotMotor2->StopMotor();
}   

void SubPivotIntake::GoUp(){
    mPivotMotor1->Set(PivotConstants::kSpeedPivot);
    mPivotMotor2->Set(-PivotConstants::kSpeedPivot);
}

void SubPivotIntake::GoDown(){
    mPivotMotor1->Set(-PivotConstants::kSpeedPivot);
    mPivotMotor2->Set(PivotConstants::kSpeedPivot);
}