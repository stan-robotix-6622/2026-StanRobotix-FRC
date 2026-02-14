// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivotIntake.h"
#include <numbers>


SubPivotIntake::SubPivotIntake() {
    mPivotMotor = new rev::spark::SparkMax{PivotConstants::kMotorPivotid1, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mFeedForward = new frc::ArmFeedforward{0_V, PivotConstants::kG, 1_V/1_rad_per_s};
    frc::SmartDashboard::PutNumber("Arm kG", PivotConstants::kG.value());

    mPivotMotorConfig = new rev::spark::SparkMaxConfig{};
    mPivotMotorConfig->Inverted(PivotConstants::kInverted);
    mPivotMotorConfig->SetIdleMode(PivotConstants::kIdleMode);
    mPivotMotor->Configure(*mPivotMotorConfig, PivotConstants::kReset, PivotConstants::kPersist);
}

// This method will be called once per scheduler run
void SubPivotIntake::Periodic() {
    frc::SmartDashboard::PutNumber("Arm Position", mPivotMotor->GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Arm Angle", GetAngle());
}

void SubPivotIntake::Stop() {
    mPivotMotor->StopMotor();
}

void SubPivotIntake::SetVoltage(double iVoltage){
    mPivotMotor->SetVoltage(units::volt_t(iVoltage));
}

void SubPivotIntake::KeepPosition()
{
    mPivotMotor->SetVoltage(units::volt_t(frc::SmartDashboard::GetNumber("Arm kG", PivotConstants::kG.value()) * cos(GetAngle())));
    // mPivotMotor->SetVoltage(PivotConstants::kG * cos(GetAngle()));
}

double SubPivotIntake::GetAngle(){
    return (mPivotMotor->GetEncoder().GetPosition() + PivotConstants::kOffset) * 2 * std::numbers::pi / PivotConstants::kGearRatio;
}