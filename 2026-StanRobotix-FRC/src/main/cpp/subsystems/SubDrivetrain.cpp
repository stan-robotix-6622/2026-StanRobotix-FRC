// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.h"

SubDrivetrain::SubDrivetrain(SubIMU* iIMU)
{
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
    mPose2dPublisher = mNTDrivetrainTable->GetStructTopic<frc::Pose2d>("Current Pose2d").Publish();
    mPose2dSubscriber = mNTDrivetrainTable->GetStructTopic<frc::Pose2d>("Current Pose2d").Subscribe(*mStartingRobotPose);

    // Initialization of the IMU
    mIMU = iIMU;

    // Initialization of the swerve kinematics with the SwerveModules' location
    mKinematics = new frc::SwerveDriveKinematics<4>{*mFrontLeftLocation, *mFrontRightLocation, *mBackLeftLocation, *mBackRightLocation};

    // Initialization of the swerve pose estimator with the kinematics, the robot's rotation, an array of the SwerveModules' position, and the robot's pose
    mPoseEstimator = new frc::SwerveDrivePoseEstimator<4>{*mKinematics, mIMU->getRotation2d(), getSwerveModulePositions(), *mStartingRobotPose};

    // Initialization des standard deviations de la vision
    visionMeasurementStdDevs = new wpi::array<double, 3>{LimelightConstants::kPoseEstimatorStandardDeviationX,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationY,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationYaw};
    mPoseEstimator->SetVisionMeasurementStdDevs(*visionMeasurementStdDevs);

    mField2d = new frc::Field2d{};
    frc::SmartDashboard::PutData("Drivetrain/Field2d", mField2d);

    // Wait for the robot to be connected to the DriverStation
    frc::DriverStation::WaitForDsConnection(0_s);
    frc2::CommandScheduler::GetInstance().Schedule(frc2::cmd::Print("The Driver Station is connected!"));
    pathplanner::AutoBuilder::configure(
        [this]()
        { return getPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose)
        { resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this]()
        { return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards)
        { driveRobotRelative(speeds, PathPlannerConstants::kPathPlannerSpeedModulation); },                                                           // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<pathplanner::PPHolonomicDriveController>(                                                                                    // PPHolonomicController is the built in path following controller for holonomic drive trains
            pathplanner::PIDConstants(PathPlannerConstants::kPTranslation, PathPlannerConstants::kITranslation, PathPlannerConstants::kDTranslation), // Translation PID constants
            pathplanner::PIDConstants(PathPlannerConstants::kPRotation, PathPlannerConstants::kIRotation, PathPlannerConstants::kDRotation)           // Rotation PID constants
            ),
        config, // The robot configuration
        []()
        {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
            frc::SmartDashboard::PutNumber("alliance color", frc::DriverStation::GetAlliance().value());
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

// This method will be called once per scheduler run
void SubDrivetrain::Periodic()
{
    // Refreshing the SwerveModules' position and states
    refreshSwerveModules();

    // Update of the robot's pose with the robot's rotation and an array of the SwerveModules' position
    mCurrentRotation2d = mIMU->getRotation2d();

    mPoseEstimator->Update(mCurrentRotation2d, getSwerveModulePositions());

    mField2d->SetRobotPose(mPoseEstimator->GetEstimatedPosition());

    // Update la rotation du robot pour la Limelight

    /* LimelightHelpers::SetRobotOrientation("", mIMU->getAngleYaw().value(), mIMU->getYawRate().value(), 0, 0, 0, 0);

    mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("");

    bool rejectCameraUpdate = false;

    if (abs(mIMU->getYawRate().value()) > 360)
    {
        rejectCameraUpdate = true;
    }
    else if (mt2.tagCount == 0)
    {
        rejectCameraUpdate = true;
    }
    else if (mt2.pose == frc::Pose2d(0_m, 0_m, 0_rad))
    {
        rejectCameraUpdate = true;
    }

    if (!rejectCameraUpdate)
    {
        mPoseEstimator->AddVisionMeasurement(mt2.pose, frc::Timer::GetFPGATimestamp());
    }*/

    // Publication de valeurs sur le NetworkTables
    mCurrentChassisSpeedsPublisher.Set(getRobotRelativeSpeeds());
    mCurrentModuleStatesPublisher.Set(getSwerveModuleStates());
    mRotation2dPublisher.Set(mCurrentRotation2d.Degrees());
    mPose2dPublisher.Set(mPoseEstimator->GetEstimatedPosition());
    resetPose(mPose2dSubscriber.Get());
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
    mDesiredChassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(DrivetrainConstants::kSpeedConstant * iX,
                                                                        DrivetrainConstants::kSpeedConstant * iY,
                                                                        DrivetrainConstants::kSpeedConstant0 * i0,
                                                                        mIMU->getRotation2d());

    // Transforming the ChassisSpeeds into four SwerveModuleState for each SwerveModule
    mDesiredSwerveStates = mKinematics->ToSwerveModuleStates(mDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br

    frc::SmartDashboard::PutNumber("Drivetrain/SetPoint", mDesiredSwerveStates[0].angle.Radians().value());
    frc::SmartDashboard::PutNumber("Drivetrain/Position", mFrontLeftModule->getModuleState().angle.Radians().value());
    mDesiredChassisSpeedsPublisher.Set(mDesiredChassisSpeeds);
    mDesiredModuleStatesPublisher.Set(mDesiredSwerveStates);

    // Setting the desired state of each SwerveModule to the corresponding SwerveModuleState
    mFrontLeftModule->setDesiredState(mDesiredSwerveStates[0], iSpeedModulation);
    mFrontRightModule->setDesiredState(mDesiredSwerveStates[1], iSpeedModulation);
    mBackLeftModule->setDesiredState(mDesiredSwerveStates[2], iSpeedModulation);
    mBackRightModule->setDesiredState(mDesiredSwerveStates[3], iSpeedModulation);
}

void SubDrivetrain::mesureSwerveFeedforward(units::volt_t iDrivingVoltage, units::volt_t iTurningVoltage)
{
    mFrontLeftModule->setDrivingVoltage(iDrivingVoltage);
    mFrontRightModule->setDrivingVoltage(iDrivingVoltage);
    mBackLeftModule->setDrivingVoltage(iDrivingVoltage);
    mBackRightModule->setDrivingVoltage(iDrivingVoltage);
    mFrontLeftModule->setTurningVoltage(iTurningVoltage);
    mFrontRightModule->setTurningVoltage(iTurningVoltage);
    mBackLeftModule->setTurningVoltage(iTurningVoltage);
    mBackRightModule->setTurningVoltage(iTurningVoltage);
    units::radian_t wCurrentTurningPosition = mFrontLeftModule->getModuleState().angle.Radians();
    units::radians_per_second_t wCurrentTurningVelocity = units::math::abs(wCurrentTurningPosition - mLastTurningPosition) / 0.020_s;
    mLastTurningPosition = wCurrentTurningPosition;
    frc::SmartDashboard::PutNumber("Drivetrain/Driving Voltage", iDrivingVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Turning Voltage", iTurningVoltage.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Driving Velocity", mFrontLeftModule->getModuleState().speed.value());
    frc::SmartDashboard::PutNumber("Drivetrain/Turning Velocity", wCurrentTurningVelocity.value());
}

frc::Pose2d SubDrivetrain::getPose()
{
    return mPoseEstimator->GetEstimatedPosition();
}

void SubDrivetrain::resetPose(frc::Pose2d iRobotPose)
{
    mPoseEstimator->ResetPose(iRobotPose);
    mIMU->setAngleYaw(iRobotPose.Rotation().Degrees());
}

frc::ChassisSpeeds SubDrivetrain::getRobotRelativeSpeeds()
{
    // Getting the current chassis speeds from the SwerveModules' state
    mCurrentChassisSpeeds = mKinematics->ToChassisSpeeds(getSwerveModuleStates());
    return mCurrentChassisSpeeds;
}

void SubDrivetrain::driveRobotRelative(frc::ChassisSpeeds iDesiredChassisSpeeds, double iSpeedModulation)
{
    // Tansforming the ChassisSpeeds into four SwerveModuleState for each SwerveModule
    mDesiredSwerveStates = mKinematics->ToSwerveModuleStates(iDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br

    // Setting the desired state of each SwerveModule to the corresponding SwerveModuleState
    mFrontLeftModule->setDesiredState(mDesiredSwerveStates[0], iSpeedModulation);
    mFrontRightModule->setDesiredState(mDesiredSwerveStates[1], iSpeedModulation);
    mBackLeftModule->setDesiredState(mDesiredSwerveStates[2], iSpeedModulation);
    mBackRightModule->setDesiredState(mDesiredSwerveStates[3], iSpeedModulation);
}