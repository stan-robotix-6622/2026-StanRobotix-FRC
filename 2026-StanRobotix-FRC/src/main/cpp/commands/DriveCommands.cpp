// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// #include <format>
#include <vector>
#include <numbers>

#include <frc/DataLogManager.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.h"
#include "commands/DriveCommands.h"

frc2::CommandPtr getFeedforwardCharacterizationCommand(SubDrivetrain* iDrivetrain)
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
                  iDrivetrain->driveRobotRelative(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
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
                  wpi::array<frc::SwerveModuleState, 4U> wModuleStates = iDrivetrain->getSwerveModuleStates();
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

frc2::CommandPtr getWheelRadiusCharacterizationCommand(SubDrivetrain* iDrivetrain)
{
    return frc2::cmd::None();
}