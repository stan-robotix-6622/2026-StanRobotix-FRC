// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivotIntake.h"


SubPivotIntake::SubPivotIntake() {
    mPivotMotor = new rev::spark::SparkMax{PivotConstants::kMotorPivotid1, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mPIDController = new frc::PIDController{PivotConstants::kP, PivotConstants::kI, PivotConstants::kD};
}



// This method will be called once per scheduler run
void SubPivotIntake::Periodic() {}


void SubPivotIntake::Stop() {
    mPivotMotor->StopMotor();
}

void SubPivotIntake::SetVoltage(double iVoltage){
    mPivotMotor->SetVoltage(units::volt_t(iVoltage));
}

void SubPivotIntake::KeepPosition()
{
    mPivotMotor->SetVoltage(units::volt_t(PivotConstants::kG * cos(GetAngle())));
}

double SubPivotIntake::GetAngle(){
    return (mPivotMotor->GetEncoder().GetPosition() + PivotConstants::kOffset) * 2 * std::numbers::pi / PivotConstants::kGearRatio;
}