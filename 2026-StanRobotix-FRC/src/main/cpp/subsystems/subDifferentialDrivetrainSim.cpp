// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDifferentialDrivetrainSim.h"

SubDifferentialDrivetrainSim::SubDifferentialDrivetrainSim(){
  std::cout << "i am heerrrrrrrrrrrrrrrrrrrrrrrrrrrrrrreeeeeeeeeeeeeeeeeeeeeeeeeee\n";
    frc::SmartDashboard::PutData("Field", &m_field);
};

// This method will be called once per scheduler run
void SubDifferentialDrivetrainSim::Periodic() 
{
    m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t(m_leftEncoder.GetDistance()),
                    units::meter_t(m_rightEncoder.GetDistance()));
    m_field.SetRobotPose(m_odometry.GetPose());
}

void SubDifferentialDrivetrainSim::SimulationPeriodic() {
  // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.
  m_driveSim.SetInputs(
    m_leftMotor.Get() * units::volt_t(frc::RobotController::GetInputVoltage()),
    m_rightMotor.Get() * units::volt_t(frc::RobotController::GetInputVoltage()));
  // Advance the model by 20 ms. Note that if you are running this
  // subsystem in a separate thread or have changed the nominal timestep
  // of TimedRobot, this value needs to match it.
  m_driveSim.Update(20_ms);
  // Update all of our sensors.
  m_leftEncoderSim.SetDistance(m_driveSim.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_driveSim.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(m_driveSim.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_driveSim.GetRightVelocity().value());
  m_gyroSim.SetAngle(-m_driveSim.GetHeading().Degrees().value());
}

void SubDifferentialDrivetrainSim::Drive(double xSpeed, double ySpeed){
  m_driveSim.SetInputs(3_V, 3_V);
  m_driveSim.Update(20_ms);
  m_leftEncoderSim.SetDistance(m_driveSim.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_driveSim.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(m_driveSim.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_driveSim.GetRightVelocity().value());
  m_gyroSim.SetAngle(-m_driveSim.GetHeading().Degrees().value());
}