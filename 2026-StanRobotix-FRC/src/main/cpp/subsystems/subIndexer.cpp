// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subIndexer.h"

subIndexer::subIndexer() {
    mTalonSRX = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX{subIndexConstants::kCANid};
};



// This method will be called once per scheduler run
void subIndexer::Periodic() {}

void subIndexer::setVoltage(units::volt_t iOutput)
{
    mTalonSRX->SetVoltage(iOutput);
};

