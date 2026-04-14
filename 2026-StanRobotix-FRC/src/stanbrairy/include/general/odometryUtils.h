#include <frc/geometry/Translation2d.h>

#include <units/angle.h>
#include <units/length.h>

namespace stanbrairy
{
  namespace odometryUtils
  {
    units::degree_t GetAngleToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation);

    units::meter_t GetDistanceToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation);
  }
}