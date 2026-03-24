// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "subsystems/SubDrivetrain.h"
#include "subsystems/SubShooter.h"
#include "ShooterLookupTable.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootDynamically
    : public frc2::CommandHelper<frc2::Command, ShootDynamically> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  ShootDynamically(SubShooter* iShooter, SubDrivetrain* iDrivetrain);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  SubDrivetrain* mDrivetrain;
  SubShooter* mShooter;

  frc::PIDController* mShooterPIDController;
  units::turns_per_second_t mCurrentVelocity;
  units::turns_per_second_t mAdjustedVelocity;
  units::turns_per_second_t mPIDAdjustment;

  units::meter_t mDistanceToTarget = 0_m;
  frc::ChassisSpeeds mRobotMovement = {0_mps, 0_mps, 0_rad_per_s};
  frc::Translation2d mTargetMovement = {0_m, 0_m};
  ShooterLookupTable::Status mShooterStatus = {0_m, 0_tps, 0_s};
};
