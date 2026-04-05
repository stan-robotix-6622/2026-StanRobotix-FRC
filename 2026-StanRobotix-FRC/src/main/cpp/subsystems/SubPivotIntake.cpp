// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivotIntake.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <numbers>

#include "Configs.h"

SubPivotIntake::SubPivotIntake() {
    mPivotMotor = new rev::spark::SparkMax{CANid::kMotorPivotID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    mEncoder = new rev::spark::SparkRelativeEncoder{mPivotMotor->GetEncoder()};
    mFeedForward = new frc::ArmFeedforward{PivotConstants::kS, PivotConstants::kG, PivotConstants::kV};

    mPivotMotor->Configure(Configs::Pivot::Config(), PivotConstants::kReset, PivotConstants::kPersist);
}

void SubPivotIntake::Periodic() {}

void SubPivotIntake::Stop()
{
	mPivotMotor->StopMotor();
}

void SubPivotIntake::SetVoltage(units::volt_t iVoltage) {
    mPivotMotor->SetVoltage(iVoltage);
}

void SubPivotIntake::SetVelocity(units::radians_per_second_t iVelocity)
{
    mPivotMotor->SetVoltage(mFeedForward->Calculate(GetAngle(), iVelocity));
}

void SubPivotIntake::KeepPosition()
{
    mPivotMotor->SetVoltage(mFeedForward->Calculate(GetAngle(), 0_rad_per_s));
}

units::radian_t SubPivotIntake::GetAngle(){
    return units::radian_t(PivotConstants::kOffset + mEncoder->GetPosition());
}

void SubPivotIntake::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("pivot");
    builder.AddDoubleProperty("position", [this] {return mEncoder->GetPosition();}, nullptr);
    builder.AddDoubleProperty("angle (radians)", [this] {return GetAngle().value();}, nullptr);
    builder.AddDoubleProperty("angle (degrees)", [this] {return units::degree_t(GetAngle()).value();}, nullptr);
    builder.AddDoubleProperty("kV", [this] {return mFeedForward->GetKv().value();}, [this] (double iKv) {return mFeedForward->SetKv(TemplateUnits::VoltageInverse<units::radians_per_second>(iKv));});
    builder.AddDoubleProperty("kG", [this] {return mFeedForward->GetKg().value();}, [this] (double iKg) {return mFeedForward->SetKg(units::volt_t(iKg));});
    builder.AddDoubleProperty("kS", [this] {return mFeedForward->GetKs().value();}, [this] (double iKs) {return mFeedForward->SetKs(units::volt_t(iKs));});
}
