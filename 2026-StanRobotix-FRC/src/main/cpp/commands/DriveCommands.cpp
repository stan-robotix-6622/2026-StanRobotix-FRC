// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// #include <format>
#include <vector>
#include <numbers>

#include <frc/DataLogManager.h>
#include <frc/filter/SlewRateLimiter.h>

#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "Constants.h"
#include "commands/DriveCommands.h"

frc2::CommandPtr DriveCommands::getFeedforwardCharacterizationCommand(SubDrivetrain* iDrivetrain)
{
    std::vector<units::radians_per_second_t> wVelocitySamples;
    std::vector<units::volt_t> wVoltageSamples;
    frc::Timer wTimer;

    return frc2::cmd::Sequence(
        // Reset data
        frc2::cmd::RunOnce(
            [wVelocitySamples, wVoltageSamples] mutable {
              wVelocitySamples.clear();
              wVoltageSamples.clear();
            }),

        // Allow modules to orient
        frc2::cmd::Run(
                [iDrivetrain] {
                  iDrivetrain->mesureSwerveFeedforward(0_V);
                },
                {iDrivetrain})
            .WithTimeout(2.0_s),

        // Start timer
        frc2::cmd::RunOnce([wTimer] mutable {wTimer.Restart();}),

        // Accelerate and gather data
        frc2::cmd::Run(
                [iDrivetrain, wTimer, wVelocitySamples, wVoltageSamples] mutable {
                  units::volt_t wVoltage = wTimer.Get() * (0.1_V / 1_s);
                  iDrivetrain->mesureSwerveFeedforward(wVoltage);
                  std::array<frc::SwerveModuleState, 4U> wModuleStates = iDrivetrain->getSwerveModuleStates();
                  units::radians_per_second_t wAverageAngularSpeed = 0_rad_per_s;
                  for (int i = 0; i < 4; i++)
                  {
                    wAverageAngularSpeed += units::radian_t(std::numbers::pi) * wModuleStates[i].speed / (ModuleConstants::kWheelPerimeter * 4);
                  }
                  wVelocitySamples.emplace_back(wAverageAngularSpeed);
                  wVoltageSamples.emplace_back(wVoltage);
                },
                {iDrivetrain})

            // When cancelled, calculate and print results
            .FinallyDo(
                [wVelocitySamples, wVoltageSamples] mutable {
                  int n = wVelocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += wVelocitySamples[i].value();
                    sumY += wVoltageSamples[i].value();
                    sumXY += wVelocitySamples[i].value() * wVoltageSamples[i].value();
                    sumX2 += wVelocitySamples[i].value() * wVelocitySamples[i].value();
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  printf("********** Drive FF Characterization Results **********");
                  printf("kS: %f", kS);
                  printf("kV: %f", kV);
                //   frc::DataLogManager::Log("********** Drive FF Characterization Results **********");
                //   frc::DataLogManager::Log(std::format("kS: {}", kS));
                //   frc::DataLogManager::Log(std::format("kV: {}", kV));
                }));
}

frc2::CommandPtr DriveCommands::getWheelRadiusCharacterizationCommand(SubDrivetrain* iDrivetrain)
{
    frc::SlewRateLimiter<units::radians_per_second> limiter{0.05_rad / 1_s / 1_s};
    WheelRadiusCharacterizationState state;

    return frc2::cmd::Parallel(
        // Drive control sequence
        frc2::cmd::Sequence(
            // Reset acceleration limiter
            frc2::cmd::RunOnce(
                [limiter] mutable {
                  limiter.Reset(0.0_rad_per_s);
                }),

            // Turn in place, accelerating up to full speed
            frc2::cmd::Run(
                [limiter, iDrivetrain] mutable {
                  units::radians_per_second_t speed = limiter.Calculate(0.25_rad_per_s);
                  iDrivetrain->driveRobotRelative(frc::ChassisSpeeds(0.0_mps, 0.0_mps, speed));
                },
                {iDrivetrain})),

        // Measurement sequence
        frc2::cmd::Sequence(
            // Wait for modules to fully orient before starting measurement
            frc2::cmd::Wait(1.0_s),

            // Record starting measurement
            frc2::cmd::RunOnce(
                [state, iDrivetrain] mutable {
                  auto wSwervePositions = iDrivetrain->getSwerveModulePositions();
                  for (int i = 0; i < 4; i++)
                  {
                    // Divide the distance traveled by the current WheelPerimeter to get the number of rotations
                    // And then convert it to radians
                    state.positions[i] = units::radian_t(std::numbers::pi) * wSwervePositions[i].distance / ModuleConstants::kWheelPerimeter;
                  }
                  state.lastAngle = iDrivetrain->getPose().Rotation();
                  state.gyroDelta = 0.0_rad;
                }),

            // Update gyro delta
            frc2::cmd::Run(
                    [state, iDrivetrain] mutable {
                      frc::Rotation2d rotation = iDrivetrain->getPose().Rotation();
                      state.gyroDelta += units::math::abs(rotation.Radians() - state.lastAngle.Radians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .FinallyDo(
                    [state, iDrivetrain] {
                      std::array<units::radian_t, 4> wPositions;
                      auto wSwervePositions = iDrivetrain->getSwerveModulePositions();
                      for (int i = 0; i < 4; i++)
                      {
                        wPositions[i] = units::radian_t(wSwervePositions[i].distance / ModuleConstants::kWheelPerimeter);
                      }
                      units::radian_t wWheelDelta;
                      for (int i = 0; i < 4; i++) {
                        wWheelDelta += units::math::abs(wPositions[i] - state.positions[i]) / 4.0;
                      }
                      units::meter_t wheelRadius =
                          (state.gyroDelta * DrivetrainConstants::kFrontLeftTranslation.Norm()) / wWheelDelta;

                      printf("********** Wheel Radius Characterization Results **********");
                      printf("Wheel Delta: %f radians", wWheelDelta);
                      printf("Gyro Delta: %f radians", state.gyroDelta);
                      printf("Wheel Radius: %f meters", wheelRadius);
                      // frc::DataLogManager::Log("********** Wheel Radius Characterization Results **********");
                      // frc::DataLogManager::Log(std::format("Wheel Delta: {} radians", wWheelDelta));
                      // frc::DataLogManager::Log(std::format("Gyro Delta: {} radians", state.gyroDelta));
                      // frc::DataLogManager::Log(std::format("Wheel Radius: {} meters", wheelRadius));
                    })));
}