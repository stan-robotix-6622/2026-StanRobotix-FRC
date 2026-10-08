// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivotIntake.h"

#include <frc/RobotBase.h>
#include <frc/Alert.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>

#include <numbers>

#include "Configs.h"
#include "RobotixLib.hpp"

SubPivotIntake::SubPivotIntake()
{
	mPivotLeaderMotor = new rev::spark::SparkMax{CANid::kLeaderMotorPivotID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mPivotFollowerMotor = new rev::spark::SparkMax{CANid::kFollowerMotorPivotID, rev::spark::SparkLowLevel::MotorType::kBrushless};
	mEncoder = new rev::spark::SparkRelativeEncoder{mPivotLeaderMotor->GetEncoder()};
	mFeedForward = new frc::ArmFeedforward{PivotConstants::kS, PivotConstants::kG, PivotConstants::kV};

	std::array<rev::REVLibError, 2> ErrorMessages = Configure();
	std::array<frc::Alert*, 2> mConfigurationErrors;
	switch (ErrorMessages[0])
	{
		case rev::REVLibError::kCANDisconnected:
			mConfigurationErrors[0] = new frc::Alert{"Pivot Leader Motor controller is Disconnected from CAN", frc::Alert::AlertType::kError};
			break;
		case rev::REVLibError::kOk:
			mConfigurationErrors[0] = new frc::Alert{"Pivot Leader Motor controller can correctly configured", frc::Alert::AlertType::kInfo};
			break;

		default:
			mConfigurationErrors[0] = new frc::Alert{"Pivot Leader Motor controller was configured with unknow error message", frc::Alert::AlertType::kWarning};
			break;
	}
	mConfigurationErrors[0]->Set(true);
	switch (ErrorMessages[1])
	{
		case rev::REVLibError::kCANDisconnected:
			mConfigurationErrors[1] = new frc::Alert{"Pivot Follower Motor controller is Disconnected from CAN", frc::Alert::AlertType::kError};
			break;
		case rev::REVLibError::kOk:
			mConfigurationErrors[1] = new frc::Alert{"Pivot Follower Motor controller can correctly configured", frc::Alert::AlertType::kInfo};
			break;

		default:
			mConfigurationErrors[1] = new frc::Alert{"Pivot Follower Motor controller was configured with unknow error message", frc::Alert::AlertType::kWarning};
			break;
	}
	mConfigurationErrors[1]->Set(true);

	// Simulation
	if (frc::RobotBase::IsSimulation()) {
		mRobotIsSimulated = true;
		mGearBox = new frc::DCMotor{frc::DCMotor::NEO(2)};
		mMotorSim = new rev::spark::SparkMaxSim{mPivotLeaderMotor, mGearBox};
		mArmPlant = new frc::LinearSystem<2, 1, 2>{frc::LinearSystemId::SingleJointedArmSystem(*mGearBox, kMOI, PivotConstants::kGearRatio)};
		mArmSim = new frc::sim::SingleJointedArmSim{*mArmPlant, *mGearBox, PivotConstants::kGearRatio, 12_in, 0_deg, 150_deg, true, units::radian_t(PivotConstants::kOffset), {0.0, 0.0}};
	}
}

void SubPivotIntake::Periodic() {}

void SubPivotIntake::Stop()
{
	mPivotLeaderMotor->StopMotor();
}

void SubPivotIntake::SetVoltage(units::volt_t iVoltage)
{
	mPivotLeaderMotor->SetVoltage(iVoltage);

	if (mRobotIsSimulated) {
		mArmSim->SetInputVoltage(iVoltage);
		mArmSim->Update(0.02_s);
		mMotorSim->iterate(mArmSim->GetVelocity().value(), 12, 0.02);
	}
}

void SubPivotIntake::SetVelocity(units::radians_per_second_t iVelocity)
{
	mPivotLeaderMotor->SetVoltage(mFeedForward->Calculate(GetAngle(), iVelocity));

	if (mRobotIsSimulated) {
		mArmSim->SetInputVoltage(mFeedForward->Calculate(GetAngle(), iVelocity));
		mArmSim->Update(0.02_s);
		mMotorSim->iterate(mArmSim->GetVelocity().value(), 12, 0.02);
	}
}

void SubPivotIntake::KeepPosition()
{
	mPivotLeaderMotor->SetVoltage(mFeedForward->Calculate(GetAngle(), 0_rad_per_s));
}

units::radian_t SubPivotIntake::GetAngle()
{
	return units::radian_t(frc::SmartDashboard::GetNumber("tunable/Offset pivot", PivotConstants::kOffset) + mEncoder->GetPosition());
}

std::array<rev::REVLibError, 2> SubPivotIntake::Configure()
{
	return {
		mPivotLeaderMotor->Configure(Configs::Pivot::LeaderConfig(), IntakeConstants::kReset, IntakeConstants::kPersist),
		mPivotFollowerMotor->Configure(Configs::Pivot::FollowerConfig(), IntakeConstants::kReset, IntakeConstants::kPersist)};
};

void SubPivotIntake::InitSendable(wpi::SendableBuilder& builder)
{
	builder.SetSmartDashboardType("pivot");
	builder.AddDoubleProperty("position", [this] { return mEncoder->GetPosition(); }, nullptr);
	builder.AddDoubleProperty("angle (radians)", [this] { return GetAngle().value(); }, nullptr);
	builder.AddDoubleProperty("angle (degrees)", [this] { return units::degree_t(GetAngle()).value(); }, nullptr);
	builder.AddDoubleProperty("kV", [this] { return mFeedForward->GetKv().value(); }, [this](double iKv) { return mFeedForward->SetKv(robotixLib::templateUnits::VoltageInverse<units::radians_per_second>(iKv)); });
	builder.AddDoubleProperty("kG", [this] { return mFeedForward->GetKg().value(); }, [this](double iKg) { return mFeedForward->SetKg(units::volt_t(iKg)); });
	builder.AddDoubleProperty("kS", [this] { return mFeedForward->GetKs().value(); }, [this](double iKs) { return mFeedForward->SetKs(units::volt_t(iKs)); });
}
