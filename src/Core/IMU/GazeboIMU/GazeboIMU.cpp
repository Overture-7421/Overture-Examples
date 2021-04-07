// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Created by ajahueym on 1/30/21.
//

#include "GazeboImu.h"

GazeboIMU::GazeboIMU(const std::string& robotName, const std::string& imuName) {
	auto robotTable = ntInstance.GetTable(robotName);
	imuTable = robotTable->GetSubTable(imuName);
	roll = imuTable->GetEntry("Roll");
	pitch = imuTable->GetEntry("Pitch");
	yaw = imuTable->GetEntry("Yaw");
}

double GazeboIMU::getRoll() {
	return roll.GetDouble(0.0);
}

double GazeboIMU::getPitch(){
	return pitch.GetDouble(0.0);
}

double GazeboIMU::getYaw(){
	return yaw.GetDouble(0.0);
}
