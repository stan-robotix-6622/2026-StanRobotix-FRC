// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.h"

#include "Constants.h"

SubDrivetrain::SubDrivetrain()
{
    frc::DataLogManager::Log("Debut initialisation du Drivetrain");
    // Initialization of the SwerveModules' location relative to the robot center
    mFrontLeftLocation  = new frc::Translation2d{DrivetrainConstants::kFrontLeftTranslation};
    mFrontRightLocation = new frc::Translation2d{DrivetrainConstants::kFrontRightTranslation};
    mBackLeftLocation   = new frc::Translation2d{DrivetrainConstants::kBackLeftTranslation};
    mBackRightLocation  = new frc::Translation2d{DrivetrainConstants::kBackRightTranslation};

    // Initialization of the SwerveModules with the motor IDs
    mFrontLeftModule  = new SwerveModule{CANid::kFrontLeftMotorID , CANid::kFrontLeftMotor550ID, false};
    mFrontRightModule = new SwerveModule{CANid::kFrontRightMotorID, CANid::kFrontRightMotor550ID, false};
    mBackLeftModule   = new SwerveModule{CANid::kBackLeftMotorID  , CANid::kBackLeftMotor550ID, true};
    mBackRightModule  = new SwerveModule{CANid::kBackRightMotorID , CANid::kBackRightMotor550ID, true};

    frc::SmartDashboard::PutData("swerve/fl module", mFrontLeftModule);
    frc::SmartDashboard::PutData("swerve/fr module", mFrontRightModule);
    frc::SmartDashboard::PutData("swerve/bl module", mBackLeftModule);
    frc::SmartDashboard::PutData("swerve/br module", mBackRightModule);

    // Initialization of the Swerve Data Publishers
    mCurrentModuleStatesPublisher = mNTDrivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Current SwerveModuleStates").Publish();
    mCurrentChassisSpeedsPublisher = mNTDrivetrainTable->GetStructTopic<frc::ChassisSpeeds>("Current ChassisSpeeds").Publish();
    mDesiredModuleStatesPublisher = mNTDrivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Desired SwerveModuleStates").Publish();
    mDesiredChassisSpeedsPublisher = mNTDrivetrainTable->GetStructTopic<frc::ChassisSpeeds>("Desired ChassisSpeeds").Publish();
    mRotation2dPublisher = mNTDrivetrainTable->GetStructTopic<frc::Rotation2d>("Current Rotation2d").Publish();
    mCurrentPose2dPublisher = mNTDrivetrainTable->GetStructTopic<frc::Pose2d>("Current Pose2d").Publish();
    mTargetPose2dPublisher = mNTDrivetrainTable->GetStructTopic<frc::Pose2d>("Target Pose2d").Publish();
    mLimelightPoseEstimatorPublisher = mNTDrivetrainTable->GetStructTopic<frc::Pose2d>("Limelight Pose Estimator").Publish();
    mCurrentPose2dSubscriber = mNTDrivetrainTable->GetStructTopic<frc::Pose2d>("Current Pose2d").Subscribe(*mStartingRobotPose);

    mLimelightName = std::string(LimelightConstants::kName);
    // Set Limelight's position on the robot
    LimelightHelpers::setCameraPose_RobotSpace(
        mLimelightName,
        LimelightConstants::kForward.value(),
        LimelightConstants::kRight.value(),
        LimelightConstants::kUp.value(),
        LimelightConstants::kRoll.value(),
        LimelightConstants::kPitch.value(),
        LimelightConstants::kYaw.value()
    );

    mIMU = new IMU{};
    frc::SmartDashboard::PutData("drivetrain/IMU", mIMU);

    mKinematics = new frc::SwerveDriveKinematics<4>{*mFrontLeftLocation, *mFrontRightLocation, *mBackLeftLocation, *mBackRightLocation};

    mPoseEstimator = new frc::SwerveDrivePoseEstimator<4>{*mKinematics, mIMU->getRotation2d(), getSwerveModulePositions(), *mStartingRobotPose};

    visionMeasurementStdDevs = new wpi::array<double, 3>{LimelightConstants::kPoseEstimatorStandardDeviationX,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationY,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationYaw};
    mPoseEstimator->SetVisionMeasurementStdDevs(*visionMeasurementStdDevs);

    mField2d = new frc::Field2d{};
    frc::SmartDashboard::PutData("drivetrain/Field2d", mField2d);
}

// This method will be called once per scheduler run
void SubDrivetrain::Periodic()
{
    refreshSwerveModules();

    mCurrentRotation2d = mIMU->getRotation2d();

    mPoseEstimator->Update(mCurrentRotation2d, getSwerveModulePositions());

    mField2d->SetRobotPose(mPoseEstimator->GetEstimatedPosition());

    LimelightHelpers::SetRobotOrientation(mLimelightName, mIMU->getAngleYaw().value(), mIMU->getYawRate().value(), 0, 0, 0, 0);

    if (LimelightConstants::kUseMegaTag2)
    {
        mLimelightPoseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(mLimelightName);
    }
    else
    {
        mLimelightPoseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue(mLimelightName);
    }

    // reject the camera update if the PoseEstimate is not valid
    bool rejectCameraUpdate = !LimelightHelpers::validPoseEstimate(mLimelightPoseEstimate);

    if (units::math::abs(mIMU->getYawRate()) > 360_deg_per_s)
    {
        rejectCameraUpdate = true;
    }
    else if (mLimelightPoseEstimate.tagCount == 0)
    {
        rejectCameraUpdate = true;
    }
    else if (mLimelightPoseEstimate.pose == frc::Pose2d(0_m, 0_m, 0_rad))
    {
        rejectCameraUpdate = true;
    }

    if (!rejectCameraUpdate)
    {
        mPoseEstimator->AddVisionMeasurement(mLimelightPoseEstimate.pose, frc::Timer::GetFPGATimestamp());
    }

    // Publication de valeurs sur le NetworkTables
    mCurrentChassisSpeedsPublisher.Set(getRobotRelativeSpeeds());
    mCurrentModuleStatesPublisher.Set(getSwerveModuleStates());
    mRotation2dPublisher.Set(mCurrentRotation2d.Degrees());
    mCurrentPose2dPublisher.Set(mPoseEstimator->GetEstimatedPosition());
    mLimelightPoseEstimatorPublisher.Set(mLimelightPoseEstimate.pose);
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
    frc::DataLogManager::Log("Start PathPlanner Configuration");
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
                frc::SmartDashboard::PutNumber("alliance color", frc::DriverStation::GetAlliance().value());
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
    frc::DataLogManager::Log("Finish Autobuilder Configuration");
    
    // Logging callback for current robot pose
    pathplanner::PathPlannerLogging::setLogCurrentPoseCallback([this](frc::Pose2d pose) {
        // Do whatever you want with the pose here
        mField2d->SetRobotPose(pose);
    });

    // Logging callback for target robot pose
    pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose) {
        // Do whatever you want with the pose here
        mField2d->GetObject("target pose")->SetPose(pose);
    });

    // Logging callback for the active path, this is sent as a vector of poses
    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses) {
        // Do whatever you want with the poses here
        mField2d->GetObject("path")->SetPoses(poses);
    });
    frc::DataLogManager::Log("Finish Pathplanner Configuration");
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
    // Creating a ChassisSpeeds from the wanted speeds and the robot's rotation
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

    // Transforming the ChassisSpeeds into four SwerveModuleState for each SwerveModule
    mDesiredSwerveStates = mKinematics->ToSwerveModuleStates(mDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br
    mKinematics->DesaturateWheelSpeeds(&mDesiredSwerveStates, DrivetrainConstants::kSpeedConstant);

    mDesiredChassisSpeedsPublisher.Set(mDesiredChassisSpeeds);
    mDesiredModuleStatesPublisher.Set(mDesiredSwerveStates);

    // Setting the desired state of each SwerveModule to the corresponding SwerveModuleState
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
    // Only change the IMU config if there are more than 1 deg of difference 
    // between the current and future rotation
    // to prevent repeated configurations of the IMU
    units::degree_t wCurrentRotation = mPoseEstimator->GetEstimatedPosition().Rotation().Degrees();
    units::degree_t wFutureRotation = iRobotPose.Rotation().Degrees();
    units::degree_t wDeltaRotation = units::math::fmod(units::math::abs(wCurrentRotation - wFutureRotation), 360_deg);
    if (wDeltaRotation >= 1_deg)
    {
        mIMU->setAngleYaw(iRobotPose.Rotation().Degrees());
    }
    // Reset the PoseEstimator's robot pose
    mPoseEstimator->ResetPose(iRobotPose);
}

void SubDrivetrain::resetIMU(units::degree_t iAngle)
{
    mIMU->setAngleYaw(iAngle);
}

IMU* SubDrivetrain::getIMU()
{
    return mIMU;
}

frc::ChassisSpeeds SubDrivetrain::getRobotRelativeSpeeds()
{
    // Getting the current chassis speeds from the SwerveModules' state
    mCurrentChassisSpeeds = mKinematics->ToChassisSpeeds(getSwerveModuleStates());
    return mCurrentChassisSpeeds;
}

void SubDrivetrain::driveRobotRelative(frc::ChassisSpeeds iDesiredChassisSpeeds)
{
    // Tansforming the ChassisSpeeds into four SwerveModuleState for each SwerveModule
    mDesiredSwerveStates = mKinematics->ToSwerveModuleStates(iDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br

    // Setting the desired state of each SwerveModule to the corresponding SwerveModuleState
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

frc::Pose2d SubDrivetrain::getClosestPoseAtDistanceFromHub(units::meter_t iHubtoRobotDistance)
{
    frc::Translation2d wOriginToRobotTranslation = standardizePose(getPose()).Translation();
    units::meter_t wRobotToHubX = FieldConstants::kHubCenterTranslation2d.X() - wOriginToRobotTranslation.X();
    units::meter_t wRobotToHubY = FieldConstants::kHubCenterTranslation2d.Y() - wOriginToRobotTranslation.Y();
    frc::Translation2d wRobotToHubTranslation = frc::Translation2d{wRobotToHubX, wRobotToHubY};
    // If the Robot is not in the alliance zone
    if (wRobotToHubTranslation.X() < 0_m)
    {
        return mPoseEstimator->GetEstimatedPosition();
    }

    frc::Translation2d wRobotToTargetTranslation = frc::Translation2d{
        wRobotToHubTranslation.Norm() - iHubtoRobotDistance,
        wRobotToHubTranslation.Angle()};

    frc::Translation2d wOriginToTargetTranslation = wOriginToRobotTranslation + wRobotToTargetTranslation;
    frc::Pose2d oOriginToTargetPose = standardizePose(frc::Pose2d{wOriginToTargetTranslation, wRobotToHubTranslation.Angle()});
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
