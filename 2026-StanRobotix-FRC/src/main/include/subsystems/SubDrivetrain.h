// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc/DataLogManager.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <units/voltage.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/angular_velocity.h>
#include "LimelightHelpers.h"
#include "subsystems/IMU.h"
#include "subsystems/SwerveModule.h"

class SubDrivetrain : public frc2::SubsystemBase {
 public:
  SubDrivetrain();

  void Periodic() override;

  void ConfigurePathplanner();

  void driveFieldRelative(float iX, float iY, float i0, double iSpeedModulation);

  void mesureSwerveFeedforward(units::volt_t iDrivingVoltage, wpi::array<frc::Rotation2d, 4> iDesiredHeadings);
  void setSwerveModuleStates(wpi::array<frc::SwerveModuleState, 4>);
  void refreshSwerveModules();
  wpi::array<frc::SwerveModuleState, 4> getSwerveModuleStates();
  wpi::array<frc::SwerveModulePosition, 4> getSwerveModulePositions();

  frc2::CommandPtr getFollowPathCommand(std::string iPathName);

  frc::ChassisSpeeds getRobotRelativeSpeeds();
  void driveRobotRelative(frc::ChassisSpeeds iSpeeds);

  frc::Pose2d getPose();
  void resetPose(frc::Pose2d iRobotPose);
  void resetIMU(units::degree_t iAngle);

  IMU* getIMU();

  frc::Pose2d getClosestPoseAtDistanceFromHub(units::meter_t iDesiredDistance);
  frc2::CommandPtr getGoToDistanceFromHubCommand(units::meter_t iDesiredDistance);
  frc::Pose2d standardizePose(frc::Pose2d iPose);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::Translation2d* mFrontLeftLocation;
  frc::Translation2d* mFrontRightLocation;
  frc::Translation2d* mBackLeftLocation;
  frc::Translation2d* mBackRightLocation;

  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> mNTDrivetrainTable = inst.GetTable("SmartDashboard/drivetrain");
  std::shared_ptr<nt::NetworkTable> mNTSwervePIDTable = inst.GetTable("SmartDashboard/swerve");

  nt::StructArrayPublisher<frc::SwerveModuleState> mCurrentModuleStatesPublisher;
  nt::StructPublisher<frc::ChassisSpeeds> mCurrentChassisSpeedsPublisher;
  nt::StructArrayPublisher<frc::SwerveModuleState> mDesiredModuleStatesPublisher;
  nt::StructPublisher<frc::ChassisSpeeds> mDesiredChassisSpeedsPublisher;
  nt::StructPublisher<frc::Rotation2d> mRotation2dPublisher;
  nt::StructPublisher<frc::Pose2d> mCurrentPose2dPublisher;
  nt::StructPublisher<frc::Pose2d> mTargetPose2dPublisher;
  nt::StructSubscriber<frc::Pose2d> mCurrentPose2dSubscriber;
  nt::StructPublisher<frc::Pose2d> mLimelightPoseEstimatorPublisher;

  // Declaring the four SwerveModule objects
  SwerveModule* mFrontLeftModule;
  SwerveModule* mFrontRightModule;
  SwerveModule* mBackLeftModule;
  SwerveModule* mBackRightModule;

  frc::SwerveDriveKinematics<4>* mKinematics;
  frc::Pose2d* mStartingRobotPose = new frc::Pose2d{2_m, 7_m, 0_rad};
  frc::SwerveDriveOdometry<4>* mOdometry;
  frc::SwerveDrivePoseEstimator<4>* mPoseEstimator;

  frc::Field2d* mField2d;

  wpi::array<double, 3>* visionMeasurementStdDevs;
  wpi::array<double, 3>* stateStdDevs;

  IMU* mIMU;

  // These attributes are used to not create new variables every time a function is called
  std::string mLimelightName;
  LimelightHelpers::PoseEstimate mLimelightPoseEstimate;
  bool rejectCameraUpdate;
  frc::ChassisSpeeds mDesiredChassisSpeeds;
  frc::ChassisSpeeds mCurrentChassisSpeeds;
  frc::Rotation2d mCurrentRotation2d;

  // The values are meant to be changed before being used
  wpi::array<frc::SwerveModuleState, 4> mDesiredSwerveStates = {frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)},
                                                                frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)},
                                                                frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)},
                                                                frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)}};

  // Load the RobotConfig from the GUI settings. You should probably
  // store this in your Constants file
  pathplanner::RobotConfig PathPlannerConfig = pathplanner::RobotConfig::fromGUISettings();
};