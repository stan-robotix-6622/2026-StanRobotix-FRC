// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/LinearFilter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include <units/current.h>

class SubClimb : public frc2::SubsystemBase {
 public:
	enum Direction {
		Up,
		Down,
		Lift
	};

	SubClimb();
	void SetSpeed(double iSpeed);
	void StopMotor();
	double GetPosition();
	frc2::CommandPtr GetClimbCommand(Direction iDirection);
	units::ampere_t GetCurrent();
	double GetCurrentFiltered();
	void SetDownPosition(double iDownPosition);
	double GetDownPosition();

	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	void Periodic() override;

 private:
	// Components (e.g. motor controllers and sensors) should generally be
	// declared private and exposed only through public methods.

	rev::spark::SparkMax* mSparkMaxLeader;
	rev::spark::SparkMax* mSparkMaxFollower;
	rev::spark::SparkRelativeEncoder* mSparkRelativeEncoder;

	frc::LinearFilter<units::ampere_t>* mHighPassFilter;
	units::ampere_t mCurrent;
	double mCurrentFiltered;
	double mDownPosition;
};
