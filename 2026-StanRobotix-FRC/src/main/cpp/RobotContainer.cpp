// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Command.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/events/EventTrigger.h>
#include <pathplanner/lib/events/PointTowardsZoneTrigger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "commands/DriveCommands.h"
#include "commands/PivotIntake.h"
#include "commands/FullIntake.h"
#include "commands/Shoot.h"
#include "commands/ShootDynamically.h"
#include "commands/ClimbUntilDown.h"

#include <iostream>

#include "Constants.h"

namespace {
  const units::meter_t DrivetrainDefaultSetpoint = 3_m;
}

RobotContainer::RobotContainer()
{
  mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  mCommandXboxControllerCopilot = new frc2::CommandXboxController{OperatorConstants::kCopilotControllerPort};
  frc::SmartDashboard::PutData("Pilot Controller", &mCommandXboxController->GetHID());
  frc::SmartDashboard::PutData("Copilot Controller", &mCommandXboxControllerCopilot->GetHID());

  frc::SmartDashboard::PutNumber("tunable/Shooter Setpoint", ShooterConstants::PIDConstants::setpoint.value());
  frc::SmartDashboard::PutNumber("tunable/Drivetrain Distance Setpoint", DrivetrainDefaultSetpoint.value());
  frc::SmartDashboard::PutNumber("tunable/Feeder Voltage", FeederConstants::kDesiredVoltage.value());
  frc::SmartDashboard::PutNumber("tunable/Time of Flight", 0);

  mSubClimb = new SubClimb;
  mSubShooter = new SubShooter{};
  frc::SmartDashboard::PutData("shooter", mSubShooter);
  mSubFeeder = new SubFeeder{};
  mDrivetrain = new SubDrivetrain{};
  frc::SmartDashboard::PutData("swerve", mDrivetrain);
  mSubIntake = new SubIntake{};
  frc::SmartDashboard::PutData("intake", mSubIntake);
  mSubPivotIntake = new SubPivotIntake{};
  frc::SmartDashboard::PutData("pivot", mSubPivotIntake);

  mDriveCommands = new DriveCommands{mDrivetrain};
  
  SetSubsystemDefaultCommands();
  RegisterCommandsPathPlanner();
  ConfigureBindings();
  ConfigureBindingsCopilot();

  mAutoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &mAutoChooser);

  mShooterStatusPublisher = mNTShooterStatusTable->GetStructArrayTopic<LookupTable::ShooterStatus>("status").Publish();
  mShooterStatusSubscriber = mNTShooterStatusTable->GetStructArrayTopic<LookupTable::ShooterStatus>("status").Subscribe(std::span<const LookupTable::ShooterStatus>());
}

void RobotContainer::SetSubsystemDefaultCommands()
{
  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
      [this]
      {
        mDrivetrain->driveFieldRelative(Deadband(-mCommandXboxController->GetLeftY(), 0.05),
                                        Deadband(-mCommandXboxController->GetLeftX(), 0.05),
                                        Deadband(-mCommandXboxController->GetRightX(), 0.05),
                                        (0.5 + (mCommandXboxController->GetRightTriggerAxis() / 2)));
      },
      {mDrivetrain}));

  mSubClimb->SetDefaultCommand(ClimbUntilDown(mSubClimb).ToPtr());
  mSubPivotIntake->SetDefaultCommand(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kUp));
}

void RobotContainer::RegisterCommandsPathPlanner()
{
  pathplanner::NamedCommands::registerCommand("Pivot-Up", PivotIntake(mSubPivotIntake, PivotIntake::StatePivotIntake::kUp).ToPtr());
  pathplanner::NamedCommands::registerCommand("Pivot-Down", PivotIntake(mSubPivotIntake, PivotIntake::StatePivotIntake::kDown).ToPtr());
  pathplanner::NamedCommands::registerCommand("Full-Intake", FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown));
  pathplanner::NamedCommands::registerCommand("GoTo3mFromHub", mDrivetrain->Defer([this]
                                                                                  { return mDrivetrain->getGoToDistanceFromHubCommand(3_m); }));
  pathplanner::NamedCommands::registerCommand("Intake", mSubIntake->getIntakeCommand());
  pathplanner::NamedCommands::registerCommand("Shoot", Shoot(mSubShooter).ToPtr());
  pathplanner::NamedCommands::registerCommand("Feed-Shooter", mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));
  pathplanner::NamedCommands::registerCommand("Unstuck-Feeder", mSubFeeder->getFeedShooterCommand(-FeederConstants::kDesiredVoltage));

  pathplanner::EventTrigger("Intake").WhileTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown)).OnTrue(frc2::cmd::Print("run Intake"));
  pathplanner::EventTrigger("Shoot").WhileTrue(Shoot(mSubShooter).ToPtr()).OnTrue(frc2::cmd::Print("run Shooter"));

  pathplanner::PointTowardsZoneTrigger("Hub").WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage)).OnTrue(frc2::cmd::Print("feed Shooter"));
}

void RobotContainer::ConfigureBindings()
{
  frc2::Trigger{[this]
                { return mDrivetrain->isTowardsHub() && mSubShooter->atDesiredVelocity(); }}
      .Debounce(0.1_s).WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));
  frc::SmartDashboard::PutBoolean("status/isFeederAutoOn", (mDrivetrain->isTowardsHub()
                  && units::math::abs(mSubShooter->getVelocity() - mSubShooter->getAdjustedVelocity()) < 0.5_tps));

  mCommandXboxController->Button(OperatorConstants::Button::X).ToggleOnTrue(mSubClimb->GetClimbCommand(SubClimb::Direction::Lift));
  mCommandXboxController->Button(OperatorConstants::kPivotDownButton).ToggleOnTrue(FullIntake::FullIntakeCommand(mSubIntake, mSubPivotIntake, PivotIntake::StatePivotIntake::kDown));
  
  mCommandXboxController->Button(OperatorConstants::kShootButton).ToggleOnTrue(Shoot(mSubShooter).ToPtr());
  mCommandXboxController->Button(OperatorConstants::kFeedButton).WhileTrue(mSubFeeder->getFeedShooterCommand(FeederConstants::kDesiredVoltage));

  mCommandXboxController->Button(OperatorConstants::kResetIMUButton).WhileTrue(frc2::cmd::RunOnce([this]
    { if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
      {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 0_deg));}
      else {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 180_deg));} }));

  mCommandXboxController->Button(OperatorConstants::kResetPoseButton).WhileTrue(frc2::cmd::RunOnce([this]
      { mDrivetrain->resetPose(SubDrivetrain::standardizePose(FieldConstants::kHubCenterPose2d)); }));

  // mCommandXboxController->Button(7).WhileTrue(mDriveCommands->getFeedforwardCharacterizationCommand());
  // mCommandXboxController->Button(8).WhileTrue(mDriveCommands->getWheelRadiusCharacterizationCommand());

  mCommandXboxController->Button(OperatorConstants::Button::Back).ToggleOnTrue(ShootDynamically(mSubShooter, mDrivetrain, mCommandXboxController).ToPtr());

  mCommandXboxController->Button(OperatorConstants::Button::LeftJoystick).OnTrue(frc2::cmd::RunOnce([this] {
    std::vector<LookupTable::ShooterStatus> vector = mShooterStatusSubscriber.Get();
    units::meter_t distance = mDrivetrain->getTranslationToHub().Norm();
    units::turns_per_second_t velocity = units::turns_per_second_t(frc::SmartDashboard::GetNumber("tunable/Shooter Setpoint", 0));
    units::second_t TOF = units::second_t(frc::SmartDashboard::GetNumber("tunable/Time of Flight", 0));
    vector.emplace_back(LookupTable::ShooterStatus{distance, velocity, TOF});
    std::cout << "ShooterStatus vector:\n";
    for (unsigned int i = 0; i < vector.size(); i++)
    {
      std::cout << "    ShooterStatus{" << vector[i].distanceToTarget.value() << "_m, "
                << vector[i].shooterVelocity.value() << "_tps, "
                << vector[i].timeOfFlight.value() << "_s},\n";
    }
    mShooterStatusPublisher.Set(vector);}));
    
  mCommandXboxController->Button(OperatorConstants::Button::Start).WhileTrue(mDrivetrain->Defer([this] { return mDrivetrain->getGoToDistanceFromHubCommand(
  (units::meter_t)frc::SmartDashboard::GetNumber("tunable/Drivetrain Distance Setpoint", DrivetrainDefaultSetpoint.value())); }));
}

void RobotContainer::ConfigureBindingsCopilot()
{
  mCommandXboxControllerCopilot->Button(OperatorConstants::Button::A).ToggleOnTrue(mSubClimb->GetClimbCommand(SubClimb::Direction::Up)).OnTrue(frc2::cmd::Print("Climb Up"));
  mCommandXboxControllerCopilot->Button(OperatorConstants::Button::B).WhileTrue(frc2::cmd::RunEnd([this] {mSubClimb->SetSpeed(0.2);}, [this] {mSubClimb->StopMotor();})).OnTrue(frc2::cmd::Print("Climb Down Manuel"));
  mCommandXboxControllerCopilot->Button(OperatorConstants::Button::X).WhileTrue(frc2::cmd::RunEnd([this] {mSubClimb->SetSpeed(-0.2);}, [this] {mSubClimb->StopMotor();})).OnTrue(frc2::cmd::Print("Climb Up Manuel"));
  mCommandXboxControllerCopilot->Button(OperatorConstants::Button::Y).WhileTrue(mSubFeeder->getFeedShooterCommand(-FeederConstants::kDesiredVoltage)).OnTrue(frc2::cmd::Print("Outfeed"));

  mCommandXboxControllerCopilot->Button(OperatorConstants::kResetIMUButton).WhileTrue(frc2::cmd::RunOnce([this]
      { if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 0_deg));}
        else {mDrivetrain->resetPose(frc::Pose2d(mDrivetrain->getPose().Translation(), 180_deg));} }));

  mCommandXboxControllerCopilot->Button(OperatorConstants::kResetPoseButton).WhileTrue(frc2::cmd::RunOnce([this]
      { mDrivetrain->resetPose(SubDrivetrain::standardizePose(frc::Pose2d(2_m, 7_m, mDrivetrain->getPose().Rotation()))); }));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return mAutoChooser.GetSelected();
}

double RobotContainer::Deadband(double iInput, double iThreshold, bool iSquared)
{
  if (abs(iInput) < iThreshold)
  {
    return 0.0;
  }
  if (!iSquared)
  {
    // ((iInput > 0) - (iInput < 0)) gives us the sign of iInput
    // Then we scale the value of iInput over the range [-1; iThreshold] or [iTheshold; 1]
    return (1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold));
  }
  // Same as above but we square the value and keep the sign of the initial iInput
  return ((iInput > 0) - (iInput < 0)) *
         (1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold)) * // Squared
         (1 / (1 - iThreshold)) * (iInput - (((iInput > 0) - (iInput < 0)) * iThreshold));
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
      status.timeLeftInPeriod = matchTime + 140_s;
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
  if (status.matchPeriod == Rebuilt::MatchPeriod::Autonomous
      || status.matchPeriod == Rebuilt::MatchPeriod::TransitionShift
      || status.matchPeriod == Rebuilt::MatchPeriod::Endgame) {
    status.hubActive = true;
  }
  else {
    if (gameData[0] == 'B') { // Blue starts inactive
      if (status.matchPeriod == Rebuilt::MatchPeriod::Shift2
          || status.matchPeriod == Rebuilt::MatchPeriod::Shift4) {
        status.hubActive = mAlliance.value() == frc::DriverStation::kBlue;
      }
      else {
        status.hubActive = mAlliance.value() == frc::DriverStation::kRed;
      }
    }
    else if (gameData[0] == 'R') { // Red starts inactive
      if (status.matchPeriod == Rebuilt::MatchPeriod::Shift2
          || status.matchPeriod == Rebuilt::MatchPeriod::Shift4) {
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
