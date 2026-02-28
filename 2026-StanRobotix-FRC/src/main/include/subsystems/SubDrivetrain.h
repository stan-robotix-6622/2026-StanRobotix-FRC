// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/RobotBase.h>
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
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <units/voltage.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/angular_velocity.h>

#include "Constants.h"
#include "LimelightHelpers.h"
#include "subsystems/SubIMU.h"
#include "subsystems/SwerveModule.h"

class SubDrivetrain : public frc2::SubsystemBase {
 public:
  SubDrivetrain(SubIMU* iIMU);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Method that drives the robot in field relative drive
  void driveFieldRelative(float iX, float iY, float i0, double iSpeedModulation);

  void refreshSwerveModules();

  void mesureSwerveFeedforward(units::volt_t iDrivingVoltage, units::volt_t iTurningVoltage);

  wpi::array<frc::SwerveModuleState, 4> getSwerveModuleStates();
  wpi::array<frc::SwerveModulePosition, 4> getSwerveModulePositions();

  frc2::CommandPtr getFollowPathCommand(std::string iPathName);

  // Method that returns a ChassisSpeeds from the robot relative speeds
  frc::ChassisSpeeds getRobotRelativeSpeeds();
  // Method that drives the robot in robot relative drive
  void driveRobotRelative(frc::ChassisSpeeds iSpeeds);

  // Method that returns the robot's pose
  frc::Pose2d getPose();
  // Method that redefines the robot's pose with its input
  void resetPose(frc::Pose2d iRobotPose);
  
  frc::Pose2d getClosestPoseAtDistanceFromHub(units::meter_t iDesiredDistance);

  frc2::CommandPtr getGoToDistanceFromHubCommand(units::meter_t iDesiredDistance);

  frc::Pose2d standardizePose(frc::Pose2d iPose);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Declaring the locations of the SwerveModules
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

  // Declaring the four SwerveModule objects
  SwerveModule* mFrontLeftModule;
  SwerveModule* mFrontRightModule;
  SwerveModule* mBackLeftModule;
  SwerveModule* mBackRightModule;

  // Declaring my swerve kinematics object
  frc::SwerveDriveKinematics<4>* mKinematics;
  // Declaring the robot starting pose object
  frc::Pose2d* mStartingRobotPose = new frc::Pose2d{2_m, 7_m, 0_rad};
  // Declaring the swerve odometry object
  frc::SwerveDriveOdometry<4>* mOdometry;
  // Declaring the pose estimator
  frc::SwerveDrivePoseEstimator<4>* mPoseEstimator;

  frc::Field2d* mField2d;

  wpi::array<double, 3>* visionMeasurementStdDevs;
  wpi::array<double, 3>* stateStdDevs;

  // Declaring the IMU object
  SubIMU* mIMU = nullptr;

  // These attributes are used to not create new variables every time a function is called
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