// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Ocupamos crear una clase para que maneje todo lo del chassis, ahorita tenemos todo en Skyline y es un pequeño desastre.
 * 
 * Tengo un video de como crear esta clase rapidamente.
 *    https://owncloud.porebazu.tech/s/yVfqF4lnuJCUTiM
 * 
 * 
 * */


#pragma once
#define M_PI 3.14159265358979323846

#include <frc2/command/SubsystemBase.h>
#include "Core/MotorHandler/EctoMotor/EctoGazeboMotor.h"
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <Core/IMU/GazeboIMU/GazeboIMU.h>

class Chassis : public frc2::SubsystemBase {
 public:
  Chassis();

  void SetVelocity(frc::ChassisSpeeds target);

  frc::DifferentialDriveWheelSpeeds GetWheelsVelocity();

  frc::ChassisSpeeds GetVelocity();

  frc::Rotation2d getHeading();  //Implementación ideal, usa las clases de rotation de WPILib

  // double getHeading(); Tambien podemos hacer que regrese un double, para cuando los chicos lo implementen por su cuenta.

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  

 private:
  void PublishTelemetry();

  const double codesPerRevWheel = 420.0 * 10.0;
  const double wheelRadius = 0.0762;

  EctoGazeboMotor leftMotor {"TankChassis_clone_0", "LeftChassis"};
  EctoGazeboMotor rightMotor {"TankChassis_clone_0", "RightChassis"};

  /**
   * Se van a ocupar dos PIDs, uno que controla la velocidad del lado izquierdo y otra del lado derecho
   */
  frc2::PIDController leftPID {0.25, 0.0, 0.0};
  frc2::PIDController rightPID {0.25, 0.0, 0.0};
  double feedForwardFactor = 0.07;
  double leftFeedforward = 0.0;
  double rightFeedforward = 0.0;

  frc::DifferentialDriveKinematics kinematics {0.6_m};
  frc::ChassisSpeeds chassisVel;
  frc::DifferentialDriveWheelSpeeds wheelsVel;
  GazeboIMU imu {"TankChassis_clone_0", "imu"};
  
};
