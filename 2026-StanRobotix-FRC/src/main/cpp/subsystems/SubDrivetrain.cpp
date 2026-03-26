// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.h"

#include "Constants.h"

SubDrivetrain::SubDrivetrain()
{
    mFrontLeftLocation  = new frc::Translation2d{DrivetrainConstants::kFrontLeftTranslation};
    mFrontRightLocation = new frc::Translation2d{DrivetrainConstants::kFrontRightTranslation};
    mBackLeftLocation   = new frc::Translation2d{DrivetrainConstants::kBackLeftTranslation};
    mBackRightLocation  = new frc::Translation2d{DrivetrainConstants::kBackRightTranslation};

    mFrontLeftModule  = new SwerveModule{CANid::kFrontLeftMotorID , CANid::kFrontLeftMotor550ID, false};
    mFrontRightModule = new SwerveModule{CANid::kFrontRightMotorID, CANid::kFrontRightMotor550ID, false};
    mBackLeftModule   = new SwerveModule{CANid::kBackLeftMotorID  , CANid::kBackLeftMotor550ID, true};
    mBackRightModule  = new SwerveModule{CANid::kBackRightMotorID , CANid::kBackRightMotor550ID, true};

    mCurrentModuleStatesPublisher = mNTDrivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Current SwerveModuleStates").Publish();
    mCurrentChassisSpeedsPublisher = mNTDrivetrainTable->GetStructTopic<frc::ChassisSpeeds>("Current ChassisSpeeds").Publish();
    mDesiredModuleStatesPublisher = mNTDrivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Desired SwerveModuleStates").Publish();
    mDesiredChassisSpeedsPublisher = mNTDrivetrainTable->GetStructTopic<frc::ChassisSpeeds>("Desired ChassisSpeeds").Publish();
    mRotation2dPublisher = mNTDrivetrainTable->GetStructTopic<frc::Rotation2d>("Current Rotation2d").Publish();
    mCurrentPose2dPublisher = mNTDrivetrainTable->GetStructTopic<frc::Pose2d>("Current Pose2d").Publish();
    mTargetPose2dPublisher = mNTDrivetrainTable->GetStructTopic<frc::Pose2d>("Target Pose2d").Publish();

    mLimelight = new Limelight{LimelightConstants::kName};
    // Set Limelight's position on the robot
    mLimelight->setCameraPosition(
        LimelightConstants::kForward,
        LimelightConstants::kRight,
        LimelightConstants::kUp,
        LimelightConstants::kRoll,
        LimelightConstants::kPitch,
        LimelightConstants::kYaw
    );

    // Initialization of the IMU
    mIMU = new IMU{};
    mIMU->reset();
    frc::SmartDashboard::PutData("drivetrain/IMU", mIMU);

    // Initialization of the swerve kinematics with the SwerveModules' location
    mKinematics = new frc::SwerveDriveKinematics<4>{*mFrontLeftLocation, *mFrontRightLocation, *mBackLeftLocation, *mBackRightLocation};
    mPoseEstimator = new frc::SwerveDrivePoseEstimator<4>{*mKinematics, mIMU->getRotation2d(), getSwerveModulePositions(), *mStartingRobotPose};

    visionMeasurementStdDevs = new wpi::array<double, 3>{LimelightConstants::kPoseEstimatorStandardDeviationX,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationY,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationYaw};
    mPoseEstimator->SetVisionMeasurementStdDevs(*visionMeasurementStdDevs);

    mField2d = new frc::Field2d{};
    frc::SmartDashboard::PutData("drivetrain/Field2d", mField2d);
}

void SubDrivetrain::Periodic()
{
    refreshSwerveModules();

    mCurrentRotation2d = mIMU->getRotation2d();
    mPoseEstimator->Update(mCurrentRotation2d, getSwerveModulePositions());
    mField2d->SetRobotPose(getPose());

    mLimelightEstimatedPose = mLimelight->getPoseEstimation(getPose(), mIMU->getYawRate());
    if (mLimelightEstimatedPose)
    {
        mPoseEstimator->AddVisionMeasurement(mLimelightEstimatedPose.value(), frc::Timer::GetFPGATimestamp());
    }

    // Publication de valeurs sur le NetworkTables
    mCurrentChassisSpeedsPublisher.Set(getRobotRelativeSpeeds());
    mCurrentModuleStatesPublisher.Set(getSwerveModuleStates());
    mRotation2dPublisher.Set(mCurrentRotation2d.Degrees());
    mCurrentPose2dPublisher.Set(mPoseEstimator->GetEstimatedPosition());
}

void SubDrivetrain::setSwerveModuleStates(wpi::array<frc::SwerveModuleState, 4> iStates)
{
    mFrontLeftModule->setDesiredState(iStates[0]);
    mFrontRightModule->setDesiredState(iStates[1]);
    mBackLeftModule->setDesiredState(iStates[2]);
    mBackRightModule->setDesiredState(iStates[3]);
}

void SubDrivetrain::ConfigurePathplanner()
{
    frc::DataLogManager::Log("Started PathPlanner Configuration");

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    pathplanner::RobotConfig PathPlannerConfig = pathplanner::RobotConfig::fromGUISettings();

    pathplanner::AutoBuilder::configure(
        [this]()
        { return getPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose)
        { resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this]()
        { return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards)
        { driveRobotRelative(speeds); },                                                           // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<pathplanner::PPHolonomicDriveController>(                                                                                    // PPHolonomicController is the built in path following controller for holonomic drive trains
            pathplanner::PIDConstants(PathPlannerConstants::kPTranslation, PathPlannerConstants::kITranslation, PathPlannerConstants::kDTranslation), // Translation PID constants
            pathplanner::PIDConstants(PathPlannerConstants::kPRotation, PathPlannerConstants::kIRotation, PathPlannerConstants::kDRotation)           // Rotation PID constants
            ),
        PathPlannerConfig, // The robot configuration
        []()
        {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
    frc::DataLogManager::Log("Finished Autobuilder Configuration");

    // Logging callback for the active path, this is sent as a vector of poses
    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses) {
        // Do whatever you want with the poses here
        mField2d->GetObject("path")->SetPoses(poses);
    });
    frc::DataLogManager::Log("Finished Pathplanner Configuration");
}

void SubDrivetrain::refreshSwerveModules()
{
    mFrontLeftModule->refreshModule();
    mFrontRightModule->refreshModule();
    mBackLeftModule->refreshModule();
    mBackRightModule->refreshModule();
}

wpi::array<frc::SwerveModuleState, 4> SubDrivetrain::getSwerveModuleStates()
{
    return wpi::array<frc::SwerveModuleState, 4>{mFrontLeftModule->getModuleState(),
                                                 mFrontRightModule->getModuleState(),
                                                 mBackLeftModule->getModuleState(),
                                                 mBackRightModule->getModuleState()};
}

wpi::array<frc::SwerveModulePosition, 4> SubDrivetrain::getSwerveModulePositions()
{
    return wpi::array<frc::SwerveModulePosition, 4>{mFrontLeftModule->getModulePosition(),
                                                    mFrontRightModule->getModulePosition(),
                                                    mBackLeftModule->getModulePosition(),
                                                    mBackRightModule->getModulePosition()};
}

void SubDrivetrain::driveFieldRelative(float iX, float iY, float i0, double iSpeedModulation)
{
    if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
    {
        mDesiredChassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(iSpeedModulation * DrivetrainConstants::kSpeedConstant * iX,
                                                                            iSpeedModulation * DrivetrainConstants::kSpeedConstant * iY,
                                                                            iSpeedModulation * DrivetrainConstants::kSpeedConstant0 * i0,
                                                                            mIMU->getRotation2d());
    }
    else
    {
        mDesiredChassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(iSpeedModulation * DrivetrainConstants::kSpeedConstant * -iX,
                                                                            iSpeedModulation * DrivetrainConstants::kSpeedConstant * -iY,
                                                                            iSpeedModulation * DrivetrainConstants::kSpeedConstant0 * i0,
                                                                            mIMU->getRotation2d());
    }

    mDesiredSwerveStates = mKinematics->ToSwerveModuleStates(mDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br
    mKinematics->DesaturateWheelSpeeds(&mDesiredSwerveStates, DrivetrainConstants::kSpeedConstant);

    mDesiredChassisSpeedsPublisher.Set(mDesiredChassisSpeeds);
    mDesiredModuleStatesPublisher.Set(mDesiredSwerveStates);

    setSwerveModuleStates(mDesiredSwerveStates);
}

void SubDrivetrain::mesureSwerveFeedforward(units::volt_t iDrivingVoltage, wpi::array<frc::Rotation2d, 4> iDesiredHeadings)
{
    mFrontLeftModule->setDrivingVoltage(iDrivingVoltage);
    mFrontRightModule->setDrivingVoltage(iDrivingVoltage);
    mBackLeftModule->setDrivingVoltage(iDrivingVoltage);
    mBackRightModule->setDrivingVoltage(iDrivingVoltage);

    mFrontLeftModule->setDesiredHeading(iDesiredHeadings[0]);
    mFrontRightModule->setDesiredHeading(iDesiredHeadings[1]);
    mBackLeftModule->setDesiredHeading(iDesiredHeadings[2]);
    mBackRightModule->setDesiredHeading(iDesiredHeadings[3]);

    frc::SmartDashboard::PutNumber("drivetrain/Driving Voltage", iDrivingVoltage.value());
    frc::SmartDashboard::PutNumber("drivetrain/Driving Velocity", mFrontLeftModule->getModuleState().speed.value());
}

frc::Pose2d SubDrivetrain::getPose()
{
    return mPoseEstimator->GetEstimatedPosition();
}

void SubDrivetrain::resetPose(frc::Pose2d iRobotPose)
{
    mPoseEstimator->ResetPosition(mIMU->getRotation2d(), getSwerveModulePositions(), iRobotPose);
}

void SubDrivetrain::resetIMU(units::degree_t iAngle)
{
    mPoseEstimator->ResetRotation(iAngle);
    mIMU->setAngleYaw(iAngle);
}

IMU* SubDrivetrain::getIMU()
{
    return mIMU;
}

frc::ChassisSpeeds SubDrivetrain::getRobotRelativeSpeeds()
{
    mCurrentChassisSpeeds = mKinematics->ToChassisSpeeds(getSwerveModuleStates());
    return mCurrentChassisSpeeds;
}

frc::ChassisSpeeds SubDrivetrain::getFieldRelativeSpeeds()
{
    mCurrentChassisSpeeds = mKinematics->ToChassisSpeeds(getSwerveModuleStates());
    return frc::ChassisSpeeds::FromRobotRelativeSpeeds(mCurrentChassisSpeeds.vx,
                                                       mCurrentChassisSpeeds.vy,
                                                       mCurrentChassisSpeeds.omega,
                                                       mIMU->getRotation2d());
}

void SubDrivetrain::driveRobotRelative(frc::ChassisSpeeds iDesiredChassisSpeeds)
{
    mDesiredSwerveStates = mKinematics->ToSwerveModuleStates(iDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br

    setSwerveModuleStates(mDesiredSwerveStates);
}

frc2::CommandPtr SubDrivetrain::getFollowPathCommand(std::string iPathName)
{
    // wPath is of type std::shared_ptr<pathplanner::PathPlannerPath>
    auto wPath = pathplanner::PathPlannerPath::fromPathFile(iPathName);

    return pathplanner::AutoBuilder::followPath(wPath);
}

frc::Pose2d SubDrivetrain::standardizePose(frc::Pose2d iPose)
{
    // mAlliance is of type std::optional<frc::DriverStation::Alliance>
    auto mAlliance = frc::DriverStation::GetAlliance();
    if (mAlliance && mAlliance.value() == frc::DriverStation::kRed)
    {
        return iPose.RotateAround(FieldConstants::kFieldCenterTranslation2d, 180_deg);
    }
    return iPose;
}

frc::Translation2d SubDrivetrain::standardizeTranslation(frc::Translation2d iTranslation)
{
    // mAlliance is of type std::optional<frc::DriverStation::Alliance>
    auto mAlliance = frc::DriverStation::GetAlliance();
    if (mAlliance && mAlliance.value() == frc::DriverStation::kRed)
    {
        return iTranslation.RotateAround(FieldConstants::kFieldCenterTranslation2d, 180_deg);
    }
    return iTranslation;
}

frc::Translation2d SubDrivetrain::getTranslationToHub()
{
  return standardizeTranslation(FieldConstants::kHubCenterTranslation2d - standardizeTranslation(getPose().Translation()));
}

frc::Pose2d SubDrivetrain::getClosestPoseAtDistanceFromHub(units::meter_t iHubtoRobotDistance)
{
    frc::Translation2d wRobotToHubTranslation = getTranslationToHub();
    // If the Robot is not in the alliance zone
    if (wRobotToHubTranslation.X() < 0_m)
    {
        return mPoseEstimator->GetEstimatedPosition();
    }

    frc::Translation2d wRobotToTargetTranslation = frc::Translation2d{
        wRobotToHubTranslation.Norm() - iHubtoRobotDistance,
        wRobotToHubTranslation.Angle()};

    frc::Translation2d wOriginToTargetTranslation = getPose().Translation() + wRobotToTargetTranslation;
    frc::Pose2d oOriginToTargetPose = frc::Pose2d{wOriginToTargetTranslation, wRobotToHubTranslation.Angle()};
    mTargetPose2dPublisher.Set(oOriginToTargetPose);
    return oOriginToTargetPose;
}

frc2::CommandPtr SubDrivetrain::getGoToDistanceFromHubCommand(units::meter_t iHubtoRobotDistance)
{
    frc::Pose2d wDesiredPose = getClosestPoseAtDistanceFromHub(iHubtoRobotDistance);

    std::vector<frc::Pose2d> wPoses{
        mPoseEstimator->GetEstimatedPosition(),
        wDesiredPose};
    std::vector<pathplanner::Waypoint> wWaypoints = pathplanner::PathPlannerPath::waypointsFromPoses(wPoses);

    pathplanner::PathConstraints wConstraints{
        PathPlannerConstants::kMaxVelocity,
        PathPlannerConstants::kMaxAcceleration,
        PathPlannerConstants::kMaxAngularVelocity,
        PathPlannerConstants::kMaxAngularAcceleration};

    // wDistanceFromHubPath is of type std::shared_ptr<pathplanner::PathPlannerPath>
    auto wDistanceFromHubPath = std::make_shared<pathplanner::PathPlannerPath>(
        wWaypoints,
        wConstraints,
        std::nullopt,                                               // The ideal starting state, this is only relevant for pre-planned paths, so can be nullopt for on-the-fly paths.
        pathplanner::GoalEndState(0.0_mps, wDesiredPose.Rotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // The path is already different depending on the Alliance color
    wDistanceFromHubPath->preventFlipping = true;

    frc2::CommandPtr wGoToPoseCommand = pathplanner::AutoBuilder::followPath(wDistanceFromHubPath);

    return wGoToPoseCommand;
}

void SubDrivetrain::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("SwerveDrive");

    builder.AddDoubleProperty("Front Left Angle", [this] {return mFrontLeftModule->getModuleState().angle.Radians().value();}, nullptr);
    builder.AddDoubleProperty("Front Left Velocity", [this] {return mFrontLeftModule->getModuleState().speed.value();}, nullptr);

    builder.AddDoubleProperty("Front Right Angle", [this] {return mFrontRightModule->getModuleState().angle.Radians().value();}, nullptr);
    builder.AddDoubleProperty("Front Right Velocity", [this] {return mFrontRightModule->getModuleState().speed.value();}, nullptr);

    builder.AddDoubleProperty("Back Left Angle", [this] {return mBackLeftModule->getModuleState().angle.Radians().value();}, nullptr);
    builder.AddDoubleProperty("Back Left Velocity", [this] {return mBackLeftModule->getModuleState().speed.value();}, nullptr);

    builder.AddDoubleProperty("Back Right Angle", [this] {return mBackRightModule->getModuleState().angle.Radians().value();}, nullptr);
    builder.AddDoubleProperty("Back Right Velocity", [this] {return mBackRightModule->getModuleState().speed.value();}, nullptr);

    builder.AddDoubleProperty("Robot Angle", [this] {return mIMU->getRotation2d().Radians().value();}, nullptr);
}