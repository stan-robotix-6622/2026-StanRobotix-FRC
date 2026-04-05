// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommands.h"

#include <frc/DataLogManager.h>
#include <frc2/command/Commands.h>

#include <numbers>
#include <vector>

#include "Constants.h"

DriveCommands::DriveCommands(SubDrivetrain* iDrivetrain)
{
	mDrivetrain = iDrivetrain;

	mSpeedLimiter = new frc::SlewRateLimiter<units::meters_per_second>{DrivetrainConstants::Commands::kMaxSpeedRampRate};

	mRotationLimiter = new frc::SlewRateLimiter<units::radians_per_second>{DrivetrainConstants::Commands::kWheelRadiusRampRate};
	mState = new WheelRadiusCharacterizationState{};

	mVelocitySamples = new std::vector<units::radians_per_second_t>{};
	mVoltageSamples = new std::vector<units::volt_t>{};
	mTimer = new frc::Timer{};
}

frc2::CommandPtr DriveCommands::getMeasureMaxAttainableSpeedCommand()
{
	return frc2::cmd::Sequence(
			// Reset acceleration limiter
			frc2::cmd::RunOnce(
					[this] {
						mSpeedLimiter->Reset(0.0_mps);
					}),
			frc2::cmd::Run(
					[this] {
						mDrivetrain->mesureSwerveFeedforward(0_V, {0_rad, 0_rad, 0_rad, 0_rad});
					},
					{mDrivetrain})
					.WithTimeout(DrivetrainConstants::Commands::kMaxSpeedStartDelay),
			frc2::cmd::Run(
					[this] {
						mDrivetrain->driveRobotRelative(
								frc::ChassisSpeeds::FromRobotRelativeSpeeds(mSpeedLimiter->Calculate(DrivetrainConstants::Commands::kMaxSpeedMaxVelocity),
																														0_mps,
																														0_rad_per_s,
																														0_deg));
					},
					{mDrivetrain})
					.FinallyDo(
							[this] {
								frc::DataLogManager::Log("Acheived Speed: " + std::to_string(mDrivetrain->getRobotRelativeSpeeds().vx.value()) + " meters per second");
							}));
}

frc2::CommandPtr DriveCommands::getFeedforwardCharacterizationCommand()
{
	return frc2::cmd::Sequence(
			// Reset data
			frc2::cmd::RunOnce(
					[this] {
						mVelocitySamples->clear();
						mVoltageSamples->clear();
					}),

			// Allow modules to orient
			frc2::cmd::Run(
					[this] {
						mDrivetrain->mesureSwerveFeedforward(0_V, {0_rad, 0_rad, 0_rad, 0_rad});
					},
					{mDrivetrain})
					.WithTimeout(DrivetrainConstants::Commands::kFeedforwartStartDelay),

			// Start timer
			frc2::cmd::RunOnce([this] { mTimer->Restart(); }),

			// Accelerate and gather data
			frc2::cmd::Run(
					[this] {
						units::volt_t wVoltage = mTimer->Get() * (DrivetrainConstants::Commands::kFeedforwardRampRate);
						mDrivetrain->mesureSwerveFeedforward(wVoltage, {0_rad, 0_rad, 0_rad, 0_rad});
						std::array<frc::SwerveModuleState, 4U> wModuleStates = mDrivetrain->getSwerveModuleStates();
						units::radians_per_second_t wAverageAngularSpeed = 0.0_rad_per_s;
						for (int i = 0; i < 4; i++) {
							wAverageAngularSpeed += units::radian_t(std::numbers::pi) * wModuleStates[i].speed / (ModuleConstants::kWheelPerimeter);
						}
						wAverageAngularSpeed /= 4;
						mVelocitySamples->emplace_back(wAverageAngularSpeed);
						mVoltageSamples->emplace_back(wVoltage);
					},
					{mDrivetrain})

					// When cancelled, calculate and print results
					.FinallyDo(
							[this] {
								int n = mVelocitySamples->size();
								double sumX = 0.0;
								double sumY = 0.0;
								double sumXY = 0.0;
								double sumX2 = 0.0;
								for (int i = 0; i < n; i++) {
									sumX += mVelocitySamples->at(i).value();
									sumY += mVoltageSamples->at(i).value();
									sumXY += mVelocitySamples->at(i).value() * mVoltageSamples->at(i).value();
									sumX2 += mVelocitySamples->at(i).value() * mVelocitySamples->at(i).value();
								}
								double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
								double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

								frc::DataLogManager::Log("********** Drive FF Characterization Results **********");
								frc::DataLogManager::Log("kS:" + std::to_string(kS));
								frc::DataLogManager::Log("kV:" + std::to_string(kV));
							}));
}

frc2::CommandPtr DriveCommands::getWheelRadiusCharacterizationCommand()
{
	mState = new WheelRadiusCharacterizationState{};

	return frc2::cmd::Parallel(
			// Drive control sequence
			frc2::cmd::Sequence(
					// Reset acceleration limiter
					frc2::cmd::RunOnce(
							[this] {
								mRotationLimiter->Reset(0.0_rad_per_s);
							}),

					// Turn in place, accelerating up to full speed
					frc2::cmd::Run(
							[this] {
								units::radians_per_second_t speed = mRotationLimiter->Calculate(DrivetrainConstants::Commands::kWheelRadiusMaxVelocity);
								mDrivetrain->driveRobotRelative(frc::ChassisSpeeds(0.0_mps, 0.0_mps, speed));
							},
							{mDrivetrain})),

			// Measurement sequence
			frc2::cmd::Sequence(
					// Wait for modules to fully orient before starting measurement
					frc2::cmd::Wait(DrivetrainConstants::Commands::kWheelRadiusMeasurementStartDelay),

					// Record starting measurement
					frc2::cmd::RunOnce(
							[this] {
								mState->positions = mDrivetrain->getSwerveModulePositions();
								mState->lastAngle = mDrivetrain->getIMU()->getRotation2d();
								mState->gyroDelta = 0.0_rad;
							}),

					// Update gyro delta
					frc2::cmd::Run(
							[this] {
								frc::Rotation2d rotation = mDrivetrain->getIMU()->getRotation2d();
								mState->gyroDelta += units::math::abs(rotation.Radians() - mState->lastAngle.Radians());
								mState->lastAngle = rotation;
							})

							// When cancelled, calculate and print results
							.FinallyDo(
									[this] {
										std::array<units::radian_t, 4> wPositions;
										wpi::array<frc::SwerveModulePosition, 4U> wSwervePositions = mDrivetrain->getSwerveModulePositions();
										for (int i = 0; i < 4; i++) {
											wPositions[i] = units::radian_t(std::numbers::pi * 2 * (wSwervePositions[i].distance - mState->positions[i].distance) / ModuleConstants::kWheelPerimeter);
										}
										units::radian_t wWheelDelta = 0_rad;
										for (int i = 0; i < 4; i++) {
											wWheelDelta += units::math::abs(wPositions[i]);
										}
										wWheelDelta /= 4;
										units::meter_t wheelRadius =
												(mState->gyroDelta * DrivetrainConstants::kFrontLeftTranslation.Norm()) / wWheelDelta;

										frc::DataLogManager::Log("********** Wheel Radius Characterization Results **********");
										frc::DataLogManager::Log("Wheel Delta: " + std::to_string(wWheelDelta.value()) + " radians");
										frc::DataLogManager::Log("Gyro Delta: " + std::to_string(mState->gyroDelta.value()) + " radians");
										frc::DataLogManager::Log("Wheel Radius: " + std::to_string(wheelRadius.value()) + " meters, " + std::to_string(units::inch_t(wheelRadius).value()) + " inches");
									})));
}
