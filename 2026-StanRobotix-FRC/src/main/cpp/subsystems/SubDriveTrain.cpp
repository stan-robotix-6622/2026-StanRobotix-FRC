// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDriveTrain.h"

SubDriveTrain::SubDriveTrain(SubIMU * iIMU)
{
    // Initialization of the SwerveModules' location relative to the robot center
    m_frontLeftLocation  = new frc::Translation2d{DriveTrainConstants::kSwerveModuleOffsetFront, DriveTrainConstants::kSwerveModuleOffsetLeft};
    m_frontRightLocation = new frc::Translation2d{DriveTrainConstants::kSwerveModuleOffsetFront, DriveTrainConstants::kSwerveModuleOffsetRight};
    m_backLeftLocation   = new frc::Translation2d{DriveTrainConstants::kSwerveModuleOffsetBack, DriveTrainConstants::kSwerveModuleOffsetLeft};
    m_backRightLocation  = new frc::Translation2d{DriveTrainConstants::kSwerveModuleOffsetBack, DriveTrainConstants::kSwerveModuleOffsetRight};

    // Initialization of the SwerveModules with the motor IDs
    m_frontLeftModule  = new SwerveModule{DriveTrainConstants::kFrontLeftMotorID , DriveTrainConstants::kFrontLeftMotor550ID, true};
    m_frontRightModule = new SwerveModule{DriveTrainConstants::kFrontRightMotorID, DriveTrainConstants::kFrontRightMotor550ID, true};
    m_backLeftModule   = new SwerveModule{DriveTrainConstants::kBackLeftMotorID  , DriveTrainConstants::kBackLeftMotor550ID, false};
    m_backRightModule  = new SwerveModule{DriveTrainConstants::kBackRightMotorID , DriveTrainConstants::kBackRightMotor550ID, false};

    // Initialization of the Swerve Data Publishers
    mCurrentModuleStatesPublisher = mNTDriveTrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Current SwerveModuleStates").Publish();
    mCurrentChassisSpeedsPublisher = mNTDriveTrainTable->GetStructTopic<frc::ChassisSpeeds>("Current ChassisSpeeds").Publish();
    mDesiredModuleStatesPublisher = mNTDriveTrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Desired SwerveModuleStates").Publish();
    mDesiredChassisSpeedsPublisher = mNTDriveTrainTable->GetStructTopic<frc::ChassisSpeeds>("Desired ChassisSpeeds").Publish();
    mRotation2dPublisher = mNTDriveTrainTable->GetStructTopic<frc::Rotation2d>("Current Rotation2d").Publish();
    mPose2dPublisher = mNTDriveTrainTable->GetStructTopic<frc::Pose2d>("Current Pose2d").Publish();
    frc::SmartDashboard::PutNumber("Drivetrain/kP", SwerveConstants::kP);
    frc::SmartDashboard::PutNumber("Drivetrain/kI", SwerveConstants::kI);
    frc::SmartDashboard::PutNumber("Drivetrain/kD", SwerveConstants::kD);
    // mPConstantSubscriber = mNTSwervePIDTable->GetDoubleTopic("kP").Subscribe(SwerveConstants::kP);
    // mIConstantSubscriber = mNTSwervePIDTable->GetDoubleTopic("kI").Subscribe(SwerveConstants::kI);
    // mDConstantSubscriber = mNTSwervePIDTable->GetDoubleTopic("kD").Subscribe(SwerveConstants::kD);

    // Initialization of the IMU
    mIMU = iIMU;

    // Initialization of the swerve kinematics with the SwerveModules' location
    m_kinematics = new frc::SwerveDriveKinematics<4>{*m_frontRightLocation, *m_frontLeftLocation, *m_backRightLocation, *m_backLeftLocation};

    // Initialization of the swerve pose estimator with the kinematics, the robot's rotation, an array of the SwerveModules' position, and the robot's pose
    m_poseEstimator = new frc::SwerveDrivePoseEstimator<4>{*m_kinematics, mIMU->getRotation2d(), getSwerveModulePositions(), *m_startingRobotPose};

	// Initialization des standard deviations de la vision
    visionMeasurementStdDevs = new wpi::array<double, 3>{LimelightConstants::kPoseEstimatorStandardDeviationX,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationY,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationYaw};
    m_poseEstimator->SetVisionMeasurementStdDevs(*visionMeasurementStdDevs);

    // pathplanner::AutoBuilder::configure(
    //     [this]()
    //     { return getPose(); }, // Robot pose supplier
    //     [this](frc::Pose2d pose)
    //     { resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
    //     [this]()
    //     { return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     [this](auto speeds, auto feedforwards)
    //     { driveRobotRelative(speeds, PathPlannerConstants::kPathPlannerSpeedModulation); },                                                           // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //     std::make_shared<pathplanner::PPHolonomicDriveController>(                                                                                    // PPHolonomicController is the built in path following controller for holonomic drive trains
    //         pathplanner::PIDConstants(PathPlannerConstants::kPTranslation, PathPlannerConstants::kITranslation, PathPlannerConstants::kDTranslation), // Translation PID constants
    //         pathplanner::PIDConstants(PathPlannerConstants::kPRotation, PathPlannerConstants::kIRotation, PathPlannerConstants::kDRotation)           // Rotation PID constants
    //         ),
    //     config, // The robot configuration
    //     []()
    //     {
    //         // Boolean supplier that controls when the path will be mirrored for the red alliance
    //         // This will flip the path being followed to the red side of the field.
    //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //         std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
    //         std::cout << alliance.value() << std::endl;
    //         if (alliance) {
    //             std::cout << alliance.value() << std::endl;
    //             return alliance.value() == frc::DriverStation::Alliance::kRed;
    //         }
    //         return false;
    //     },
    //     this // Reference to this subsystem to set requirements
    // );
}

// This method will be called once per scheduler run
void SubDriveTrain::Periodic()
{
    // Refreshing the SwerveModules' position and states
    m_frontLeftModule->refreshModule();
    m_frontRightModule->refreshModule();
    m_backLeftModule->refreshModule();
    m_backRightModule->refreshModule();

    refreshSwervePID();

    // // Update of the robot's pose with the robot's rotation and an array of the SwerveModules' position
    mCurrentRotation2d = mIMU->getRotation2d();
    
    m_poseEstimator->Update(mCurrentRotation2d, getSwerveModulePositions());

    // Update la rotation du robot pour la Limelight
  
    /* LimelightHelpers::SetRobotOrientation("", mIMU->getAngleYaw(), mIMU->getYawRate(), 0, 0, 0, 0);

    mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("");

    bool rejectCameraUpdate = false;

    if (abs(mIMU->getYawRate()) > 360)
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
        m_poseEstimator->AddVisionMeasurement(mt2.pose, frc::Timer::GetFPGATimestamp());
    }*/
    
    // Publication de valeurs sur le NetworkTables
    mCurrentChassisSpeedsPublisher.Set(getRobotRelativeSpeeds());
    mRotation2dPublisher.Set(mCurrentRotation2d.Degrees());
    mPose2dPublisher.Set(m_poseEstimator->GetEstimatedPosition());
    mCurrentModuleStatesPublisher.Set(getSwerveModuleStates());
}

void SubDriveTrain::refreshSwervePID()
{
    double wP = frc::SmartDashboard::GetNumber("Drivetrain/kP", SwerveConstants::kP);
    double wI = frc::SmartDashboard::GetNumber("Drivetrain/kI", SwerveConstants::kI);
    double wD = frc::SmartDashboard::GetNumber("Drivetrain/kD", SwerveConstants::kD);

    m_frontLeftModule->setPIDValues(wP, wI, wD);
    m_frontRightModule->setPIDValues(wP, wI, wD);
    m_backLeftModule->setPIDValues(wP, wI, wD);
    m_backRightModule->setPIDValues(wP, wI, wD);
}

wpi::array<frc::SwerveModuleState, 4> SubDriveTrain::getSwerveModuleStates()
{
    return wpi::array<frc::SwerveModuleState, 4> {m_frontLeftModule->getModuleState(),
                                                  m_frontRightModule->getModuleState(),
                                                  m_backLeftModule->getModuleState(),
                                                  m_backRightModule->getModuleState()};
}

wpi::array<frc::SwerveModulePosition, 4> SubDriveTrain::getSwerveModulePositions()
{
    return wpi::array<frc::SwerveModulePosition, 4> {m_frontLeftModule->getModulePosition(),
                                                     m_frontRightModule->getModulePosition(),
                                                     m_backLeftModule->getModulePosition(),
                                                     m_backRightModule->getModulePosition()};
}

void SubDriveTrain::driveFieldRelative(float iX, float iY, float i0, double iSpeedModulation)
{
    // Creating a ChassisSpeeds from the wanted speeds and the robot's rotation
    mDesiredChassisSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(DriveTrainConstants::kSpeedConstant * iX,
                                                                        DriveTrainConstants::kSpeedConstant * iY,
                                                                        DriveTrainConstants::kSpeedConstant0 * i0,
                                                                        mIMU->getRotation2d());

    // Transforming the ChassisSpeeds into four SwerveModuleState for each SwerveModule
    mSwerveDesiredStates = m_kinematics->ToSwerveModuleStates(mDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br

    mDesiredChassisSpeedsPublisher.Set(mDesiredChassisSpeeds);
    mDesiredModuleStatesPublisher.Set(mSwerveDesiredStates);
    
    frc::SmartDashboard::PutNumber("Drivetrain/SetPoint", mSwerveDesiredStates[0].angle.Radians().value());
    frc::SmartDashboard::PutNumber("Drivetrain/Position", m_frontLeftModule->getModuleState().angle.Radians().value());

    // Setting the desired state of each SwerveModule to the corresponding SwerveModuleState
    m_frontLeftModule->setDesiredState(mSwerveDesiredStates[0], iSpeedModulation);
    m_frontRightModule->setDesiredState(mSwerveDesiredStates[1], iSpeedModulation);
    m_backLeftModule->setDesiredState(mSwerveDesiredStates[2], iSpeedModulation);
    m_backRightModule->setDesiredState(mSwerveDesiredStates[3], iSpeedModulation);
}

frc::Pose2d SubDriveTrain::getPose()
{
    return m_poseEstimator->GetEstimatedPosition();
}

void SubDriveTrain::resetPose(frc::Pose2d iRobotPose)
{
    m_poseEstimator->ResetPose(iRobotPose);
}

frc::ChassisSpeeds SubDriveTrain::getRobotRelativeSpeeds()
{
    // Getting the current chassis speeds from the SwerveModules' state
    mCurrentChassisSpeeds = m_kinematics->ToChassisSpeeds(getSwerveModuleStates());
    return mCurrentChassisSpeeds;
}

void SubDriveTrain::driveRobotRelative(frc::ChassisSpeeds iDesiredChassisSpeeds, double iSpeedModulation)
{
    // Tansforming the ChassisSpeeds into four SwerveModuleState for each SwerveModule
    mSwerveDesiredStates = m_kinematics->ToSwerveModuleStates(iDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br

    // Setting the desired state of each SwerveModule to the corresponding SwerveModuleState
    m_frontLeftModule->setDesiredState(mSwerveDesiredStates[0], iSpeedModulation);
    m_frontRightModule->setDesiredState(mSwerveDesiredStates[1], iSpeedModulation);
    m_backLeftModule->setDesiredState(mSwerveDesiredStates[2], iSpeedModulation);
    m_backRightModule->setDesiredState(mSwerveDesiredStates[3], iSpeedModulation);
}