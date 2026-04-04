// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"

#include <frc2/command/Commands.h>

#include "Configs.h"

SubIntake::SubIntake() {
    mIntakeMotor = new rev::spark::SparkMax(CANid::kMotorIntakeID, rev::spark::SparkLowLevel::MotorType::kBrushless);

    mEncoder = new rev::spark::SparkRelativeEncoder{mIntakeMotor->GetEncoder()};
 
    mIntakeMotor->Configure(Configs::Intake::Config(), IntakeConstants::kReset, IntakeConstants::kPersist);
}

void SubIntake::Periodic() {}

void SubIntake::Stop() {
    mIntakeMotor->StopMotor();
}

void SubIntake::SetVoltage(double iVoltage)
{
    mIntakeMotor->SetVoltage(units::volt_t(iVoltage));
}

void SubIntake::SetSpeed(double iSpeed)
{
    mIntakeMotor->Set(iSpeed);
}

frc2::CommandPtr SubIntake::getIntakeCommand() {
  return frc2::cmd::RunEnd(
    [this]
    {
      SetSpeed(IntakeConstants::kSpeed);
    },

    [this]
    {
      Stop();
    },
    {}
  );
}

bool SubIntake::isIntakeOn()
{
  return mIntakeMotor->GetAppliedOutput() > 0;
}

void SubIntake::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("intake");
    builder.AddDoubleProperty("velocity rpm", [this] {return mEncoder->GetVelocity();}, nullptr);
    builder.AddDoubleProperty("wheel perimeter speed", [this] {return mEncoder->GetVelocity() * IntakeConstants::kWheelRadius.value() * 2 * std::numbers::pi / 60;}, nullptr);
}