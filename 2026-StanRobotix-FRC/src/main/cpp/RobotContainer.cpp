// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/RobotState.h>
#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/events/EventTrigger.h>
#include <pathplanner/lib/events/PointTowardsZoneTrigger.h>

#include <iostream>
#include <string>
#include <vector>

#include "Constants.h"

// #include "commands/Climb.h"
// #include "commands/ClimbUntilDown.h"
#include "commands/DriveCommands.h"
#include "commands/FullIntake.h"
#include "commands/PivotIntake.h"
#include "commands/Shoot.h"
#include "commands/ShootVariable.h"
#include "commands/ShootInPlace.h"
#include "commands/ShootDynamically.h"
#include "commands/PointTowardsHub.h"
#include "commands/PointTowardsZone.h"

namespace
{
	const units::meter_t DrivetrainDefaultSetpoint = 3_m;
}	 // namespace

RobotContainer::RobotContainer()
{
	mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
	mCommandXboxControllerCopilot = new frc2::CommandGenericHID{OperatorConstants::kCopilotControllerPort};
	frc::SmartDashboard::PutData("Pilot Controller", &mCommandXboxController->GetHID());
	// frc::SmartDashboard::PutData("Copilot Controller", &mCommandXboxControllerCopilot->GetHID());

	frc::SmartDashboard::PutNumber("tunable/Shooter Setpoint", ShooterConstants::PIDConstants::setpoint.value());
	frc::SmartDashboard::PutNumber("tunable/Drivetrain Distance Setpoint", DrivetrainDefaultSetpoint.value());
	frc::SmartDashboard::PutNumber("tunable/Feeder Voltage", FeederConstants::kDesiredVoltage.value());
	frc::SmartDashboard::PutNumber("tunable/Time of Flight", 0);
	frc::SmartDashboard::PutNumber("tunable/Pivot kg", PivotConstants::kG.value());
	frc::SmartDashboard::PutNumber("tunable/Shooter tolerance", 1);
	frc::SmartDashboard::PutNumber("tunable/Offset pivot", PivotConstants::kOffset);

	// mSubClimb = new SubClimb;
	mSubShooter = new SubShooter{};
	mSubFeeder = new SubFeeder{};
	mDrivetrain = new SubDrivetrain{};
	mSubIntake = new SubIntake{};
	mSubPivotIntake = new SubPivotIntake{};

	frc::SmartDashboard::PutData("swerve", mDrivetrain);
	frc::SmartDashboard::PutData("shooter", mSubShooter);
	frc::SmartDashboard::PutData("intake", mSubIntake);
	frc::SmartDashboard::PutData("pivot", mSubPivotIntake);

	mDriveCommands = new DriveCommands{mDrivetrain};

	mIsInAllianceZoneTrigger = new frc2::Trigger{[this] {return mDrivetrain->isInAllianceZone();}};

	SetSubsystemDefaultCommands();
	RegisterCommandsPathPlanner();
	ConfigureBindings();
	ConfigureBindingsCopilot();

	mAutoChooser = pathplanner::AutoBuilder::buildAutoChooserFilter([this] 
			(const pathplanner::PathPlannerAuto& autoCommand)
			{return autoCommand.GetName().starts_with("Comp");});
	frc::SmartDashboard::PutData("Auto Chooser", &mAutoChooser);

	// mShooterStatusPublisher = mNTShooterStatusTable->GetStructArrayTopic<LookupTable::ShooterStatus>("status array").Publish();
	// mShooterStatusSubscriber = mNTShooterStatusTable->GetStructArrayTopic<LookupTable::ShooterStatus>("status array").Subscribe(std::span<const LookupTable::ShooterStatus>());
}

void RobotContainer::SetSubsystemDefaultCommands()
{
	mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
			[this] {
				mDrivetrain->driveFieldRelative(Deadband(-mCommandXboxController->GetLeftY(), 0.05),
																				Deadband(-mCommandXboxController->GetLeftX(), 0.05),
																				Deadband(-mCommandXboxController->GetRightX(), 0.05),
																				(0.5 + (mCommandXboxController->GetRightTriggerAxis() / 2)));
			},
			{mDrivetrain}));

	// mSubClimb->SetDefaultCommand(ClimbUntilDown(mSubClimb).ToPtr().WithTimeout(3_s));

	mSubPivotIntake->SetDefaultCommand(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kUp));
	// mSubPivotIntake->SetDefaultCommand(frc2::cmd::Run([this] {mSubPivotIntake->SetVoltage(units::volt_t(frc::SmartDashboard::GetNumber("tunable/Pivot kg", PivotConstants::kG.value()) * cos(mSubPivotIntake->GetAngle().value())));}, {mSubPivotIntake}));
}

void RobotContainer::RegisterCommandsPathPlanner()
{
	pathplanner::NamedCommands::registerCommand("Full-Intake-Down", FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown).AlongWith(frc2::cmd::Print("Intake down")));
	pathplanner::NamedCommands::registerCommand("Full-Intake-In", FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kIn).AlongWith(frc2::cmd::Print("Pivot in")));
	pathplanner::NamedCommands::registerCommand("Full-Intake-Up", FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kUp).AlongWith(frc2::cmd::Print("Pivot up")));
	pathplanner::NamedCommands::registerCommand("Intake", mSubIntake->getIntakeCommand(IntakeConstants::kSpeed).AlongWith(frc2::cmd::Print("Intake")));
	pathplanner::NamedCommands::registerCommand("Shoot", Shoot(mSubShooter).ToPtr().AlongWith(frc2::cmd::Print("Shoot")));
	pathplanner::NamedCommands::registerCommand("Shoot-Variable", ShootVariable(mSubShooter, mDrivetrain).ToPtr().AlongWith(frc2::cmd::Print("Shoot Variable")));
	pathplanner::NamedCommands::registerCommand("Feed-Shooter", mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage).AlongWith(frc2::cmd::Print("Feed")));
	pathplanner::NamedCommands::registerCommand("Out-Feed", mSubFeeder->getFeedShooterCommand(-FeederConstants::kDesiredVoltage).AlongWith(frc2::cmd::Print("OutFeed")));
	pathplanner::NamedCommands::registerCommand("Point-Hub", PointTowardsHub(mDrivetrain).ToPtr().AlongWith(frc2::cmd::Print("Point to hub")));

	pathplanner::EventTrigger("Full-Intake-Down").WhileTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown)).OnTrue(frc2::cmd::Print("run Intake"));
	pathplanner::EventTrigger("Shoot").WhileTrue(Shoot(mSubShooter).ToPtr()).OnTrue(frc2::cmd::Print("run Shooter"));
	pathplanner::EventTrigger("Shoot-Variable").WhileTrue(ShootVariable(mSubShooter, mDrivetrain).ToPtr()).OnTrue(frc2::cmd::Print("run Shooter-Variable"));

	(pathplanner::PointTowardsZoneTrigger("Hub") && frc2::Trigger([this] {return mDrivetrain->isTowardsHub() && mSubShooter->atDesiredVelocity();})).WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage)).OnTrue(frc2::cmd::Print("feed Shooter"));
}

void RobotContainer::ConfigureBindings()
{
	frc2::Trigger{[this] {
		if (frc::RobotState::IsAutonomous()) {
			return mSubShooter->atDesiredVelocity() && mDrivetrain->isInAllianceZone();
		} else {
			return mDrivetrain->isTowardsHub() && mSubShooter->atDesiredVelocity() && mDrivetrain->isInAllianceZone()
			;}}}.Debounce(0.1_s)
			.WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));

	// mCommandXboxController->Button(OperatorConstants::Button::X).WhileTrue(Climb(mSubClimb, SubClimb::Direction::Lift).ToPtr());
	mCommandXboxController->Button(OperatorConstants::Button::A).WhileTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown));

	mCommandXboxController->Button(OperatorConstants::Button::Y).WhileTrue(Shoot(mSubShooter).ToPtr());
	mCommandXboxController->Button(OperatorConstants::Button::B).WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));

	// mCommandXboxController->Button(OperatorConstants::Button::RightBumper).OnTrue(frc2::cmd::RunOnce([this] { if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
  //     {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 0_deg));}
  //     else {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 180_deg));} }));

	mCommandXboxController->Button(OperatorConstants::Button::LeftBumper).WhileTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kIn));

	// mCommandXboxController->Button(OperatorConstants::Button::Start).OnTrue(frc2::cmd::RunOnce([this] {
  //   std::vector<LookupTable::ShooterStatus> vector = mShooterStatusSubscriber.Get();
  //   units::meter_t distance = mDrivetrain->getTranslationToHub().Norm();
  //   units::turns_per_second_t velocity = units::turns_per_second_t(frc::SmartDashboard::GetNumber("tunable/Shooter Setpoint", 0));
  //   units::second_t TOF = units::second_t(frc::SmartDashboard::GetNumber("tunable/Time of Flight", 0));
  //   vector.emplace_back(LookupTable::ShooterStatus{distance, velocity, TOF});
  //   std::cout << "ShooterLookupTable new values:\n";
  //   for (unsigned int i = 0; i < vector.size(); i++)
  //   {
	// 		std::cout << "		ShooterStatus{" << vector[i].distanceToTarget.value() << "_m, "
	// 		<< vector[i].shooterVelocity.value() << "_tps, "
	// 		<< vector[i].timeOfFlight.value() << "_s}";
	// 		if (i != vector.size()) {
	// 			std::cout << ",";
	// 		}
	// 		std::cout << "\n";
	// 	}
	// 	std::cout << "}\n";
  //   mShooterStatusPublisher.Set(vector); }));

	(*mIsInAllianceZoneTrigger && mCommandXboxController->Button(OperatorConstants::Button::Start))
	.WhileTrue(ShootInPlace(mSubShooter, mDrivetrain).ToPtr());

	mCommandXboxController->Button(OperatorConstants::Button::Back).WhileTrue(frc2::cmd::Run([this] {mSubShooter->setVoltage(8_V);}));

	// mCommandXboxController->Button(7).WhileTrue(mDriveCommands->getFeedforwardCharacterizationCommand());
	// mCommandXboxController->Button(8).WhileTrue(mDriveCommands->getWheelRadiusCharacterizationCommand());

	// (*mIsInAllianceZoneTrigger && mCommandXboxController->Button(OperatorConstants::Button::Back))
	// 		.WhileTrue(ShootDynamically(mSubShooter, mDrivetrain, mCommandXboxController).ToPtr());

	// mCommandXboxController->Button(OperatorConstants::Button::Start).WhileTrue(mDrivetrain->Defer([this] { return mDrivetrain->getGoToDistanceFromHubCommand(
	// 																																																					 (units::meter_t)frc::SmartDashboard::GetNumber("tunable/Drivetrain Distance Setpoint", DrivetrainDefaultSetpoint.value())); }));
}

void RobotContainer::ConfigureBindingsCopilot()
{
	// mCommandXboxControllerCopilot->Button(OperatorConstants::Button::A).WhileTrue(Climb(mSubClimb, SubClimb::Direction::Up).ToPtr()).OnTrue(frc2::cmd::Print("Climb Up"));
	// mCommandXboxControllerCopilot->Button(OperatorConstants::Button::B).WhileTrue(mSubClimb->RunEnd([this] { mSubClimb->SetSpeed(0.2); }, [this] { mSubClimb->StopMotor(); })).OnTrue(frc2::cmd::Print("Climb Down Manuel"));
	// mCommandXboxControllerCopilot->Button(OperatorConstants::Button::X).WhileTrue(mSubClimb->RunEnd([this] { mSubClimb->SetSpeed(-0.2); }, [this] { mSubClimb->StopMotor(); })).OnTrue(frc2::cmd::Print("Climb Up Manuel"));
	mCommandXboxControllerCopilot->Button(OperatorConstants::Button::Y).WhileTrue(mSubFeeder->getFeedShooterCommand(-FeederConstants::kDesiredVoltage)).OnTrue(frc2::cmd::Print("Outfeed"));
	mCommandXboxControllerCopilot->Button(OperatorConstants::Button::B).WhileTrue(frc2::cmd::Parallel(PivotIntake(mSubPivotIntake, PivotIntake::StatePivotIntake::kDown).ToPtr(), mSubIntake->getIntakeCommand(-IntakeConstants::kSpeed)));
	mCommandXboxControllerCopilot->Button(OperatorConstants::Button::X).WhileTrue(frc2::cmd::Parallel(Shoot(mSubShooter).ToPtr(), PointTowardsZone(mDrivetrain).ToPtr()));
	mCommandXboxControllerCopilot->Button(OperatorConstants::Button::Y).WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage)).OnTrue(frc2::cmd::Print("Feed"));

	mCommandXboxControllerCopilot->Button(OperatorConstants::kResetIMUButton).WhileTrue(frc2::cmd::RunOnce([this] { if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 0_deg));}
        else {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 180_deg));} }));

	// mCommandXboxControllerCopilot->Button(OperatorConstants::kResetPoseButton).WhileTrue(frc2::cmd::RunOnce([this] { mDrivetrain->resetPose(SubDrivetrain::standardizePose(frc::Pose2d(2_m, 7_m, mDrivetrain->getPose().Rotation()))); }));
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
	return mAutoChooser.GetSelected();
}

double RobotContainer::Deadband(double iInput, double iThreshold, bool iSquared)
{
	if (abs(iInput) < iThreshold) {
		return 0.0;
	}
	if (!iSquared) {
		// ((iInput > 0) - (iInput < 0)) gives us the sign of iInput
		// Then we scale the value of iInput over the range [-1; iThreshold] or [iTheshold; 1]
		return (1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold));
	}
	// Same as above but we square the value and keep the sign of the initial iInput
	return frc::CopyDirectionPow((1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold)), 2);
}

Rebuilt::MatchStatus RobotContainer::getMatchStatus()
{
	std::string gameData = frc::DriverStation::GetGameSpecificMessage();
	units::second_t matchTime = frc::DriverStation::GetMatchTime();
	// mAlliance is of type std::optional<frc::DriverStation::Alliance>
	auto mAlliance = frc::DriverStation::GetAlliance();
	Rebuilt::MatchStatus status;
	status.timeLeftInMatch = matchTime;

	if (matchTime <= 30_s) {
		if (gameData[0] != 'B' && gameData[0] != 'R') {
			status.timeLeftInMatch = matchTime + 140_s;
			status.timeLeftInPeriod = matchTime;
			status.matchPeriod = Rebuilt::MatchPeriod::Autonomous;
			status.matchPeriodName = "Autonomous";
		}
		else {
			status.timeLeftInPeriod = matchTime;
			status.matchPeriod = Rebuilt::MatchPeriod::Endgame;
			status.matchPeriodName = "Endgame";
		}
	}
	else {
		status.timeLeftInPeriod = units::math::fmod(matchTime - 30_s, 25_s);
	}
	if (matchTime >= 130_s) {
		status.matchPeriod = Rebuilt::MatchPeriod::TransitionShift;
		status.matchPeriodName = "Transition Shift";
	}
	else if (matchTime >= 105_s) {
		status.matchPeriod = Rebuilt::MatchPeriod::Shift1;
		status.matchPeriodName = "Shift 1";
	}
	else if (matchTime >= 80_s) {
		status.matchPeriod = Rebuilt::MatchPeriod::Shift2;
		status.matchPeriodName = "Shift 2";
	}
	else if (matchTime >= 55_s) {
		status.matchPeriod = Rebuilt::MatchPeriod::Shift3;
		status.matchPeriodName = "Shift 3";
	}
	else if (matchTime >= 30_s) {
		status.matchPeriod = Rebuilt::MatchPeriod::Shift4;
		status.matchPeriodName = "Shift 4";
	}
	if (status.matchPeriod == Rebuilt::MatchPeriod::Autonomous || status.matchPeriod == Rebuilt::MatchPeriod::TransitionShift || status.matchPeriod == Rebuilt::MatchPeriod::Endgame) {
		status.hubActive = true;
	}
	else {
		if (gameData[0] == 'B') {	 // Blue starts inactive
			if (status.matchPeriod == Rebuilt::MatchPeriod::Shift2 || status.matchPeriod == Rebuilt::MatchPeriod::Shift4) {
				status.hubActive = mAlliance.value() == frc::DriverStation::kBlue;
			}
			else {
				status.hubActive = mAlliance.value() == frc::DriverStation::kRed;
			}
		}
		else if (gameData[0] == 'R') {	// Red starts inactive
			if (status.matchPeriod == Rebuilt::MatchPeriod::Shift2 || status.matchPeriod == Rebuilt::MatchPeriod::Shift4) {
				status.hubActive = mAlliance.value() == frc::DriverStation::kRed;
			}
			else {
				status.hubActive = mAlliance.value() == frc::DriverStation::kBlue;
			}
		}
		else {
			status.hubActive = false;
		}
	}
	return status;
}
