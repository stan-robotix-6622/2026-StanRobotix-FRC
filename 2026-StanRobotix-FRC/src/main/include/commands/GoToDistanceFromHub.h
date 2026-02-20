// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/length.h>
#include <units/angle.h>

#include <numbers>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "Constants.h"
#include "subsystems/SubDrivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class GoToDistanceFromHub
    : public frc2::CommandHelper<frc2::Command, GoToDistanceFromHub> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  GoToDistanceFromHub(SubDrivetrain * iDrivetrain, units::meter_t iDesiredDistance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc::Pose2d getClosestPose(frc::Pose2d iCurrentPose, units::meter_t iHubtoRobotDistance);



 private:
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> mNTVectorsTable = inst.GetTable("SmartDashboard/Vectors");
  nt::StructPublisher<frc::Pose2d> mPose2dPublisher;
  nt::StructPublisher<frc::Translation2d> mOriginToRobotPublisher;
  nt::StructPublisher<frc::Translation2d> mRobotToTargetPublisher;
  nt::StructPublisher<frc::Translation2d> mRobotToHubPublisher;

  nt::StructPublisher<frc::Pose2d> mCurrent;

  units::meter_t mDesiredDistance;

  frc::Pose2d mDesiredPose;

  SubDrivetrain * mDrivetrain = nullptr;

  std::optional<frc2::CommandPtr> mGoToPoseCommand;
};
