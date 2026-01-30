// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
// #include <networktables/DoubleTopic.h>
// #include <pathplanner/lib/auto/AutoBuilder.h>
// #include <pathplanner/lib/config/RobotConfig.h>
// #include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include "Constants.h"
// #include "LimelightHelpers.h"
#include "subsystems/SubIMU.h"
#include "subsystems/SwerveModule.h"
#include "subsystems/SwerveModuleSim.h"

class SubDriveTrain : public frc2::SubsystemBase {
 public:
  SubDriveTrain(SubIMU * iIMU);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

// Method that drives the robot in field relative drive
  void driveFieldRelative(float iX, float iY, float i0, double iSpeedModulation);

  void refreshSwervePID();

  wpi::array<frc::SwerveModuleState, 4> getSwerveModuleStates();
  wpi::array<frc::SwerveModulePosition, 4> getSwerveModulePositions();

// Method that returns a ChassisSpeeds from the robot relative speeds
  frc::ChassisSpeeds getRobotRelativeSpeeds();
// Method that drives the robot in robot relative drive
  void driveRobotRelative(frc::ChassisSpeeds iSpeeds, double SpeedModulation);

// Method that returns the robot's pose
  frc::Pose2d getPose();
// Method that redefines the robot's pose with its input
  void resetPose(frc::Pose2d iRobotPose);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Declaring the locations of the SwerveModules
  frc::Translation2d * m_frontLeftLocation;
  frc::Translation2d * m_frontRightLocation;
  frc::Translation2d * m_backLeftLocation;
  frc::Translation2d * m_backRightLocation;

  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> mNTDriveTrainTable = inst.GetTable("DriveTrain");
  std::shared_ptr<nt::NetworkTable> mNTSwervePIDTable = inst.GetTable("Swerve");

  nt::StructArrayPublisher<frc::SwerveModuleState> m_currentModuleStatesPublisher;
  nt::StructPublisher<frc::ChassisSpeeds> m_currentChassisSpeedsPublisher;
  nt::StructPublisher<frc::Rotation2d> m_rotation2dPublisher;
  nt::StructPublisher<frc::Pose2d> m_pose2dPublisher;
  // nt::DoubleSubscriber mPConstantSubscriber;
  // nt::DoubleSubscriber mIConstantSubscriber;
  // nt::DoubleSubscriber mDConstantSubscriber;

  // Declaring the four SwerveModule objects
  SwerveModuleSim * m_frontLeftModule;
  SwerveModuleSim * m_frontRightModule;
  SwerveModuleSim * m_backLeftModule;
  SwerveModuleSim * m_backRightModule;

  // Declaring my swerve kinematics object
  frc::SwerveDriveKinematics<4> * m_kinematics;
  // Declaring the robot starting pose object
  frc::Pose2d * m_startingRobotPose = new frc::Pose2d{0_m, 0_m, 0_rad};
  // Declaring the swerve odometry object
  frc::SwerveDriveOdometry<4> * m_odometry;
  // Declaring the pose estimator
  frc::SwerveDrivePoseEstimator<4> * m_poseEstimator;

  wpi::array<double, 3> * visionMeasurementStdDevs;
  wpi::array<double, 3> * stateStdDevs;

  // Declaring the IMU object
  SubIMU * mIMU = nullptr;

  // These attributes are used to not create new variables every time a function is called
  // LimelightHelpers::PoseEstimate mt2;
  bool rejectCameraUpdate;

  frc::ChassisSpeeds mDesiredChassisSpeeds;
  frc::ChassisSpeeds mCurrentChassisSpeeds;

  frc::Rotation2d mCurrentRotation2d;

  // The values are meant to be changed before being used
  wpi::array<frc::SwerveModuleState, 4> mSwerveDesiredStates = {frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)},
                                                                frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)},
                                                                frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)},
                                                                frc::SwerveModuleState{0_mps, frc::Rotation2d(0_rad)}};
  // Load the RobotConfig from the GUI settings. You should probably
  // store this in your Constants file
  // pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();
};