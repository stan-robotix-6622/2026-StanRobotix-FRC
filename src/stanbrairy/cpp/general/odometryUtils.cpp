#include "general/odometryUtils.h"

#include <units/math.h>

units::degree_t stanbrairy::odometryUtils::GetAngleToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation)
{
  frc::Translation2d wTranslationToTarget = iTargetTranslation - iCurrentTranslation;
  return wTranslationToTarget.Angle().Degrees();
}

units::meter_t stanbrairy::odometryUtils::GetDistanceToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation)
{
  frc::Translation2d wTranslationToTarget = iTargetTranslation - iCurrentTranslation;
  return wTranslationToTarget.Norm();
}