// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class GazeboIMU {
public:
	GazeboIMU(const std::string& robotName, const std::string& imuName);
	double getRoll();
	double getPitch();
	double getYaw();
private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> imuTable;
	nt::NetworkTableEntry roll, pitch, yaw;
};

