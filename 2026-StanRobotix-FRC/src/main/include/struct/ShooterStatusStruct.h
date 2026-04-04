// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>
#include <wpi/struct/Struct.h>

#include "ShooterLookupTable.h"

template <>
struct WPILIB_DLLEXPORT wpi::Struct<LookupTable::ShooterStatus> {
  static constexpr std::string_view GetTypeName() { return "ShooterStatus"; }
  static constexpr size_t GetSize() { return 24; }
  static constexpr std::string_view GetSchema() {
    return "double distance;double velocity;double TOF";
  }

  static LookupTable::ShooterStatus Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const LookupTable::ShooterStatus& value);
};

static_assert(wpi::StructSerializable<LookupTable::ShooterStatus>);
