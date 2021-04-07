// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Chassis.h"
#include <frc/smartdashboard/SmartDashboard.h>

Chassis::Chassis(){
    leftMotor.setFeedbackMode(MotorFeedbackMode::QuadEncoder);
    rightMotor.setFeedbackMode(MotorFeedbackMode::QuadEncoder);

    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
};

void Chassis::SetVelocity(frc::ChassisSpeeds target){
    frc::DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(target);
    
    leftPID.SetSetpoint(wheelSpeeds.left.to<double>());
    rightPID.SetSetpoint(wheelSpeeds.right.to<double>());

    leftFeedforward = wheelSpeeds.left.to<double>() * feedForwardFactor;
    rightFeedforward = wheelSpeeds.right.to<double>() * feedForwardFactor;
}

frc::DifferentialDriveWheelSpeeds Chassis::GetWheelsVelocity(){
    const double leftMotorVel = leftMotor.getVelocity() / codesPerRevWheel * 2 * M_PI * wheelRadius;
    const double rightMotorVel = rightMotor.getVelocity() / codesPerRevWheel * 2 * M_PI * wheelRadius;

    frc::DifferentialDriveWheelSpeeds wheelSpeeds;

    wheelSpeeds.left = units::meters_per_second_t(leftMotorVel);
    wheelSpeeds.right = units::meters_per_second_t(rightMotorVel);
    return wheelSpeeds;
}

frc::ChassisSpeeds Chassis::GetVelocity(){
    return chassisVel;
}


// This method will be called once per scheduler run
void Chassis::Periodic() {
    wheelsVel = GetWheelsVelocity();
    chassisVel = kinematics.ToChassisSpeeds(wheelsVel);

    double leftOut = leftPID.Calculate(wheelsVel.left.to<double>()) + leftFeedforward;
    double rightOut = rightPID.Calculate(wheelsVel.right.to<double>()) + rightFeedforward;

    leftMotor.set(leftOut);
    rightMotor.set(rightOut);
    PublishTelemetry();
}


void Chassis::PublishTelemetry(){
    frc::SmartDashboard::PutNumber("TankChassis0/LeftWheelVel", wheelsVel.left.to<double>());
    frc::SmartDashboard::PutNumber("TankChassis0/RightWheelVel", wheelsVel.right.to<double>());

    frc::SmartDashboard::PutNumber("TankChassis0/LinearVel", chassisVel.vx.to<double>());
    frc::SmartDashboard::PutNumber("TankChassis0/AngularVel", chassisVel.omega.to<double>());
}

frc::Rotation2d Chassis::getHeading(){
    return frc::Rotation2d(units::radian_t(imu.getYaw()));
}

//double Chassis::getHeading(){
//    return imu.getYaw();
//}
