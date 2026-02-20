// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GoToDistanceFromHub.h"

GoToDistanceFromHub::GoToDistanceFromHub(SubDrivetrain * iDrivetrain, units::meter_t iDesiredDistance)
{
  mDrivetrain = iDrivetrain;
  mDesiredDistance = iDesiredDistance;

  frc::SmartDashboard::PutData("commands/go to pose command ", mGoToPoseCommand.value().get());

  mPose2dPublisher = mNTVectorsTable->GetStructTopic<frc::Pose2d>("Target Pose").Publish();
  mOriginToRobotPublisher = mNTVectorsTable->GetStructTopic<frc::Translation2d>("Origin to Robot").Publish();
  mRobotToTargetPublisher = mNTVectorsTable->GetStructTopic<frc::Translation2d>("Robot to Target").Publish();
  mRobotToHubPublisher = mNTVectorsTable->GetStructTopic<frc::Translation2d>("Robot to Hub").Publish();

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(mDrivetrain);
}

frc::Pose2d GoToDistanceFromHub::getClosestPose(frc::Pose2d iCurrentPose, units::meter_t iHubtoRobotDistance)
{
  frc::Translation2d wOriginToRobotTranslation = iCurrentPose.Translation();
  mOriginToRobotPublisher.Set(wOriginToRobotTranslation);
  units::meter_t wRobotToHubX = HubConstants::kHubCenterTranslation2d.X() - wOriginToRobotTranslation.X();
  units::meter_t wRobotToHubY = HubConstants::kHubCenterTranslation2d.Y() - wOriginToRobotTranslation.Y();
  frc::Translation2d wRobotToHubTranslation = frc::Translation2d{wRobotToHubX, wRobotToHubY};
  mRobotToHubPublisher.Set(wRobotToHubTranslation);
  // If the Robot is not in the alliance zone
  if (wRobotToHubTranslation.X() < 0_m)
  {
    return iCurrentPose;
  }
  
  frc::Translation2d wRobotToTargetTranslation = frc::Translation2d{
    wRobotToHubTranslation.Norm() - iHubtoRobotDistance, 
    wRobotToHubTranslation.Angle()};
    mRobotToTargetPublisher.Set(wRobotToTargetTranslation);
    
    frc::Translation2d wOriginToTargetTranslation = wOriginToRobotTranslation + wRobotToTargetTranslation; 
    frc::Pose2d oOriginToTargetPose = frc::Pose2d{wOriginToTargetTranslation, wRobotToTargetTranslation.Angle()};
  mPose2dPublisher.Set(oOriginToTargetPose);
  return oOriginToTargetPose;
}

// Called when the command is initially scheduled.
void GoToDistanceFromHub::Initialize()
{
  mDesiredPose = getClosestPose(mDrivetrain->getPose(), mDesiredDistance);
  
  std::vector<frc::Pose2d> wPoses{
    mDrivetrain->getPose(),
    mDesiredPose};
    std::vector<pathplanner::Waypoint> wWaypoints = pathplanner::PathPlannerPath::waypointsFromPoses(wPoses);
    
    pathplanner::PathConstraints wConstraints{
      PathPlannerConstants::kMaxVelocity,
      PathPlannerConstants::kMaxAcceleration, 
      PathPlannerConstants::kMaxAngularVelocity, 
    PathPlannerConstants::kMaxAngularAcceleration};

  auto wDistanceFromHubPath = std::make_shared<pathplanner::PathPlannerPath>(
    wWaypoints,
    wConstraints,
    std::nullopt, // The ideal starting state, this is only relevant for pre-planned paths, so can be nullopt for on-the-fly paths.
    pathplanner::GoalEndState(0.0_mps, mDesiredPose.Rotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );
  
  mGoToPoseCommand = pathplanner::AutoBuilder::followPath(wDistanceFromHubPath);
  
  if (mGoToPoseCommand)
  {
    frc2::CommandScheduler::GetInstance().Schedule(mGoToPoseCommand.value());
  }
}

// Called repeatedly when this Command is scheduled to run
void GoToDistanceFromHub::Execute() {}

// Called once the command ends or is interrupted.
void GoToDistanceFromHub::End(bool interrupted)
{
  if (interrupted)
  {
    if (mGoToPoseCommand)
    {
      frc2::CommandScheduler::GetInstance().Cancel(mGoToPoseCommand.value());
    }
  }
}

// Returns true when the command should end.
bool GoToDistanceFromHub::IsFinished() {
  return false;
}
