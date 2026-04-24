// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Configs.h"

SwerveModule::SwerveModule(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInverted, bool iTurningInverted)
{
	mDrivingMotor = new rev::spark::SparkMax{iDrivingMotorID, ModuleConstants::kDrivingMotorType};
	mTurningMotor = new rev::spark::SparkMax{iTurningMotorID, ModuleConstants::kTurningMotorType};

	// Simulation
	if (frc::RobotBase::IsSimulation()) {
		mRobotIsSimulated = true;
		mTurningGearBox = new frc::DCMotor{frc::DCMotor::NEO550()};
		mDrivingGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
		mTurningMotorSim = new rev::spark::SparkMaxSim{mTurningMotor, mTurningGearBox};
		mDrivingMotorSim = new rev::spark::SparkMaxSim{mDrivingMotor, mDrivingGearBox};
	}

	mDrivingMotor->Configure(Configs::SwerveModule::DrivingConfig(iDrivingInverted),
													 ModuleConstants::kDrivingResetMode,
													 ModuleConstants::kDrivingPersistMode);
	mTurningMotor->Configure(Configs::SwerveModule::TurningConfig(iTurningInverted),
													 ModuleConstants::kTurningResetMode,
													 ModuleConstants::kTurningPersistMode);

	// Initialization of the motors' ClosedLoopController
	mTurningClosedLoopController = new rev::spark::SparkClosedLoopController{mTurningMotor->GetClosedLoopController()};
	mDrivingClosedLoopController = new rev::spark::SparkClosedLoopController{mDrivingMotor->GetClosedLoopController()};

	mDrivingEncoder = new rev::spark::SparkRelativeEncoder{mDrivingMotor->GetEncoder()};
	mTurningEncoder = new rev::spark::SparkRelativeEncoder{mTurningMotor->GetEncoder()};
	mTurningAbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{mTurningMotor->GetAbsoluteEncoder()};

	refreshModule();
	seedEncoder();
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState)
{
	mTurningCurrentAngle = frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition()));
	mOptimizedState = iDesiredState;
	mOptimizedState.Optimize(mTurningCurrentAngle);
	mOptimizedState.CosineScale(mTurningCurrentAngle);

	mTurningClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value(), ModuleConstants::kTurningClosedLoopControlType);
	mDrivingClosedLoopController->SetSetpoint(mOptimizedState.speed.value(), ModuleConstants::kDrivingClosedLoopControlType);

	if (mRobotIsSimulated) {
		mTurningMotorSim->iterate((mOptimizedState.angle.Radians().value() - mTurningMotorSim->GetPosition()) / 0.02, 12, 0.02);
		mDrivingMotorSim->iterate(mOptimizedState.speed.value(), 12, 0.02);
	}
}

void SwerveModule::setDesiredHeading(frc::Rotation2d iDesiredHeading)
{
	mTurningClosedLoopController->SetSetpoint(iDesiredHeading.Radians().value(), ModuleConstants::kTurningClosedLoopControlType);

	if (mRobotIsSimulated) {
		mTurningMotorSim->iterate((iDesiredHeading.Radians().value() - mTurningMotorSim->GetPosition()) / 0.02, 12, 0.02);
	}
}

void SwerveModule::setTurningVoltage(units::volt_t iVoltage)
{
	mTurningMotor->SetVoltage(iVoltage);
}

void SwerveModule::setDrivingVoltage(units::volt_t iVoltage)
{
	mDrivingMotor->SetVoltage(iVoltage);
}

frc::SwerveModuleState SwerveModule::getModuleState()
{
	return mModuleState;
}

frc::SwerveModulePosition SwerveModule::getModulePosition()
{
	return mModulePosition;
}

units::radians_per_second_t SwerveModule::getTurningVelocity()
{
	return units::radians_per_second_t(mTurningAbsoluteEncoder->GetVelocity());
}

void SwerveModule::seedEncoder()
{
	mTurningEncoder->SetPosition(mTurningAbsoluteEncoder->GetPosition());
}

void SwerveModule::refreshModule()
{
	mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mDrivingEncoder->GetVelocity()),
																				frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition()))};
	mModulePosition = frc::SwerveModulePosition{units::meter_t(mDrivingEncoder->GetPosition()),
																							frc::Rotation2d(units::radian_t(mTurningAbsoluteEncoder->GetPosition()))};
}

void SwerveModule::InitSendable(wpi::SendableBuilder& builder)
{
	builder.SetSmartDashboardType("swerve/module");
	builder.AddDoubleProperty("turning velocity", [this] { return mTurningAbsoluteEncoder->GetVelocity(); }, nullptr);
	builder.AddDoubleProperty("turning position", [this] { return mTurningAbsoluteEncoder->GetPosition(); }, nullptr);
	builder.AddDoubleProperty("driving velocity", [this] { return mDrivingEncoder->GetVelocity(); }, nullptr);
}
