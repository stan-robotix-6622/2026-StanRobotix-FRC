// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.h"

#include <iostream>

SubDrivetrain::SubDrivetrain(SubIMU * iIMU)
{
    // Initialization of the SwerveModules' location relative to the robot center
    m_frontLeftLocation  = new frc::Translation2d{DrivetrainConstants::kSwerveModuleOffsetFront, DrivetrainConstants::kSwerveModuleOffsetLeft};
    m_frontRightLocation = new frc::Translation2d{DrivetrainConstants::kSwerveModuleOffsetFront, DrivetrainConstants::kSwerveModuleOffsetRight};
    m_backLeftLocation   = new frc::Translation2d{DrivetrainConstants::kSwerveModuleOffsetBack, DrivetrainConstants::kSwerveModuleOffsetLeft};
    m_backRightLocation  = new frc::Translation2d{DrivetrainConstants::kSwerveModuleOffsetBack, DrivetrainConstants::kSwerveModuleOffsetRight};

    // Initialization of the SwerveModules with the motor IDs
    if (frc::RobotBase::IsReal())
    {    
        std::cout << "The robot is real" << std::endl;
        m_frontLeftModule  = new SwerveModule{DrivetrainConstants::kFrontLeftMotorID , DrivetrainConstants::kFrontLeftMotor550ID, true};
        m_frontRightModule = new SwerveModule{DrivetrainConstants::kFrontRightMotorID, DrivetrainConstants::kFrontRightMotor550ID, true};
        m_backLeftModule   = new SwerveModule{DrivetrainConstants::kBackLeftMotorID  , DrivetrainConstants::kBackLeftMotor550ID, false};
        m_backRightModule  = new SwerveModule{DrivetrainConstants::kBackRightMotorID , DrivetrainConstants::kBackRightMotor550ID, false};
    }
    else
    {
        std::cout << "The robot is simulated" << std::endl;
        m_frontLeftModuleSim  = new SwerveModuleSim{DrivetrainConstants::kFrontLeftMotorID , DrivetrainConstants::kFrontLeftMotor550ID, false, true};
        m_frontRightModuleSim = new SwerveModuleSim{DrivetrainConstants::kFrontRightMotorID, DrivetrainConstants::kFrontRightMotor550ID, false, true};
        m_backLeftModuleSim   = new SwerveModuleSim{DrivetrainConstants::kBackLeftMotorID  , DrivetrainConstants::kBackLeftMotor550ID, true, true};
        m_backRightModuleSim  = new SwerveModuleSim{DrivetrainConstants::kBackRightMotorID , DrivetrainConstants::kBackRightMotor550ID, true, true};
    }
    

    // Initialization of the Swerve Data Publishers
    m_currentModuleStatesPublisher = mNTDriveTrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Current SwerveModuleStates").Publish();
    m_desiredModuleStatesPublisher = mNTDriveTrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Desired SwerveModuleStates").Publish();
    m_currentChassisSpeedsPublisher = mNTDriveTrainTable->GetStructTopic<frc::ChassisSpeeds>("Current ChassisSpeeds").Publish();
    m_desiredChassisSpeedsPublisher = mNTDriveTrainTable->GetStructTopic<frc::ChassisSpeeds>("Desired ChassisSpeeds").Publish();
    m_rotation2dPublisher = mNTDriveTrainTable->GetStructTopic<frc::Rotation2d>("Current Rotation2d").Publish();
    m_pose2dPublisher = mNTDriveTrainTable->GetStructTopic<frc::Pose2d>("Current Pose2d").Publish();
    frc::SmartDashboard::PutNumber("Drivetrain/kP", ModuleConstants::kTurningP);
    frc::SmartDashboard::PutNumber("Drivetrain/kI", ModuleConstants::kTurningI);
    frc::SmartDashboard::PutNumber("Drivetrain/kD", ModuleConstants::kTurningD);
    // mPConstantSubscriber = mNTSwervePIDTable->GetDoubleTopic("kP").Subscribe(ModuleConstants::kTurningP);
    // mIConstantSubscriber = mNTSwervePIDTable->GetDoubleTopic("kI").Subscribe(ModuleConstants::kTurningI);
    // mDConstantSubscriber = mNTSwervePIDTable->GetDoubleTopic("kD").Subscribe(ModuleConstants::kTurningD);

    // Initialization of the IMU
    mIMU = iIMU;

    // Initialization of the swerve kinematics with the SwerveModules' location
    m_kinematics = new frc::SwerveDriveKinematics<4>{*m_frontLeftLocation, *m_frontRightLocation, *m_backLeftLocation, *m_backRightLocation};

    // Initialization of the swerve pose estimator with the kinematics, the robot's rotation, an array of the SwerveModules' position, and the robot's pose
    m_poseEstimator = new frc::SwerveDrivePoseEstimator<4>{*m_kinematics, mIMU->getRotation2d(), getSwerveModulePositions(), *m_startingRobotPose};
	// Initialization des standard deviations de la vision
    visionMeasurementStdDevs = new wpi::array<double, 3>{LimelightConstants::kPoseEstimatorStandardDeviationX,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationY,
                                                         LimelightConstants::kPoseEstimatorStandardDeviationYaw};
    m_poseEstimator->SetVisionMeasurementStdDevs(*visionMeasurementStdDevs);

    mField2d = new frc::Field2d{};

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
void SubDrivetrain::Periodic()
{
    // Refreshing the SwerveModules' position and states
    refreshSwerveModules();

    refreshSwervePID();

    // Update of the robot's pose with the robot's rotation and an array of the SwerveModules' position
    if (frc::RobotBase::IsReal())
    {
        mCurrentRotation2d = mIMU->getRotation2d();
    }
    else
    {
        mIMU->setSimYawRate(m_kinematics->ToChassisSpeeds(getSwerveModuleStates()).omega);
        mIMU->setSimAngleYaw(units::degree_t(mIMU->getAngleYaw() + mIMU->getYawRate() * 0.2));
        mCurrentRotation2d = mIMU->getRotation2d();
    }

    m_poseEstimator->Update(mCurrentRotation2d, getSwerveModulePositions());

    mField2d->SetRobotPose(m_poseEstimator->GetEstimatedPosition());
    frc::SmartDashboard::PutData("Drivetrain/Field2d", mField2d);

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
    m_currentChassisSpeedsPublisher.Set(getRobotRelativeSpeeds());
    m_rotation2dPublisher.Set(mCurrentRotation2d.Degrees());
    m_pose2dPublisher.Set(m_poseEstimator->GetEstimatedPosition());
    m_currentModuleStatesPublisher.Set(getSwerveModuleStates());
}

void SubDrivetrain::refreshSwerveModules()
{
    if (frc::RobotBase::IsReal())
    {
        m_frontLeftModule->refreshModule();
        m_frontRightModule->refreshModule();
        m_backLeftModule->refreshModule();
        m_backRightModule->refreshModule();
    }
    else
    {
        m_frontLeftModuleSim->refreshModule();
        m_frontRightModuleSim->refreshModule();
        m_backLeftModuleSim->refreshModule();
        m_backRightModuleSim->refreshModule();
    }
}

void SubDrivetrain::refreshSwervePID()
{
    double wP = frc::SmartDashboard::GetNumber("Drivetrain/kP", ModuleConstants::kTurningP);
    double wI = frc::SmartDashboard::GetNumber("Drivetrain/kI", ModuleConstants::kTurningI);
    double wD = frc::SmartDashboard::GetNumber("Drivetrain/kD", ModuleConstants::kTurningD);

    if (frc::RobotBase::IsReal())
    {
        m_frontLeftModule->setPIDValues(wP, wI, wD);
        m_frontRightModule->setPIDValues(wP, wI, wD);
        m_backLeftModule->setPIDValues(wP, wI, wD);
        m_backRightModule->setPIDValues(wP, wI, wD);
    }
    else
    {
        m_frontLeftModuleSim->setPIDValues(wP, wI, wD);
        m_frontRightModuleSim->setPIDValues(wP, wI, wD);
        m_backLeftModuleSim->setPIDValues(wP, wI, wD);
        m_backRightModuleSim->setPIDValues(wP, wI, wD);
    }
}

wpi::array<frc::SwerveModuleState, 4> SubDrivetrain::getSwerveModuleStates()
{
    if (frc::RobotBase::IsReal())
    {
        return wpi::array<frc::SwerveModuleState, 4> {m_frontLeftModule->getModuleState(),
                                                      m_frontRightModule->getModuleState(),
                                                      m_backLeftModule->getModuleState(),
                                                      m_backRightModule->getModuleState()};
    }
    else
    {
        return wpi::array<frc::SwerveModuleState, 4> {m_frontLeftModuleSim->getModuleState(),
                                                      m_frontRightModuleSim->getModuleState(),
                                                      m_backLeftModuleSim->getModuleState(),
                                                      m_backRightModuleSim->getModuleState()};
    }
}

wpi::array<frc::SwerveModulePosition, 4> SubDrivetrain::getSwerveModulePositions()
{
    if (frc::RobotBase::IsReal())
    {
        return wpi::array<frc::SwerveModulePosition, 4>{m_frontLeftModule->getModulePosition(),
                                                        m_frontRightModule->getModulePosition(),
                                                        m_backLeftModule->getModulePosition(),
                                                        m_backRightModule->getModulePosition()};
    }
    else
    {
        return wpi::array<frc::SwerveModulePosition, 4>{m_frontLeftModuleSim->getModulePosition(),
                                                        m_frontRightModuleSim->getModulePosition(),
                                                        m_backLeftModuleSim->getModulePosition(),
                                                        m_backRightModuleSim->getModulePosition()};
    }
}

void SubDrivetrain::driveFieldRelative(float iX, float iY, float i0, double iSpeedModulation)
{
    // Creating a ChassisSpeeds from the wanted speeds and the robot's rotation
    mDesiredChassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(DrivetrainConstants::kSpeedConstant * iX,
                                                                        DrivetrainConstants::kSpeedConstant * iY,
                                                                        DrivetrainConstants::kSpeedConstant0 * i0,
                                                                        mIMU->getRotation2d());

    // Transforming the ChassisSpeeds into four SwerveModuleState for each SwerveModule
    mDesiredSwerveStates = m_kinematics->ToSwerveModuleStates(mDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br
    
    frc::SmartDashboard::PutNumber("Drivetrain/SetPoint", mDesiredSwerveStates[0].angle.Radians().value());
    m_desiredChassisSpeedsPublisher.Set(mDesiredChassisSpeeds);
    m_desiredModuleStatesPublisher.Set(mDesiredSwerveStates);
    // Setting the desired state of each SwerveModule to the corresponding SwerveModuleState
    if (frc::RobotBase::IsReal())
    {
        frc::SmartDashboard::PutNumber("Drivetrain/Position", m_frontLeftModule->getModuleState().angle.Radians().value());
        m_frontLeftModule->setDesiredState(mDesiredSwerveStates[0], iSpeedModulation);
        m_frontRightModule->setDesiredState(mDesiredSwerveStates[1], iSpeedModulation);
        m_backLeftModule->setDesiredState(mDesiredSwerveStates[2], iSpeedModulation);
        m_backRightModule->setDesiredState(mDesiredSwerveStates[3], iSpeedModulation);
    }
    else
    {
        frc::SmartDashboard::PutNumber("Drivetrain/Position", m_frontLeftModuleSim->getModuleState().angle.Radians().value());
        m_frontLeftModuleSim->setDesiredState(mDesiredSwerveStates[0], iSpeedModulation);
        m_frontRightModuleSim->setDesiredState(mDesiredSwerveStates[1], iSpeedModulation);
        m_backLeftModuleSim->setDesiredState(mDesiredSwerveStates[2], iSpeedModulation);
        m_backRightModuleSim->setDesiredState(mDesiredSwerveStates[3], iSpeedModulation);
    }
}

frc::Pose2d SubDrivetrain::getPose()
{
    return m_poseEstimator->GetEstimatedPosition();
}

void SubDrivetrain::resetPose(frc::Pose2d iRobotPose)
{
    m_poseEstimator->ResetPose(iRobotPose);
}

frc::ChassisSpeeds SubDrivetrain::getRobotRelativeSpeeds()
{
    // Getting the current chassis speeds from the SwerveModules' state
    mCurrentChassisSpeeds = m_kinematics->ToChassisSpeeds(getSwerveModuleStates());
    return mCurrentChassisSpeeds;
}

void SubDrivetrain::driveRobotRelative(frc::ChassisSpeeds iDesiredChassisSpeeds, double iSpeedModulation)
{
    // Tansforming the ChassisSpeeds into four SwerveModuleState for each SwerveModule
    mDesiredSwerveStates = m_kinematics->ToSwerveModuleStates(iDesiredChassisSpeeds); // The array has in order: fl, fr, bl, br

    // Setting the desired state of each SwerveModule to the corresponding SwerveModuleState
    if (frc::RobotBase::IsReal())
    {
        m_frontLeftModule->setDesiredState(mDesiredSwerveStates[0], iSpeedModulation);
        m_frontRightModule->setDesiredState(mDesiredSwerveStates[1], iSpeedModulation);
        m_backLeftModule->setDesiredState(mDesiredSwerveStates[2], iSpeedModulation);
        m_backRightModule->setDesiredState(mDesiredSwerveStates[3], iSpeedModulation);
    }
    else
    {
        m_frontLeftModuleSim->setDesiredState(mDesiredSwerveStates[0], iSpeedModulation);
        m_frontRightModuleSim->setDesiredState(mDesiredSwerveStates[1], iSpeedModulation);
        m_backLeftModuleSim->setDesiredState(mDesiredSwerveStates[2], iSpeedModulation);
        m_backRightModuleSim->setDesiredState(mDesiredSwerveStates[3], iSpeedModulation);
    }
}