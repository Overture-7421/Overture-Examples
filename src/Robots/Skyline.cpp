#include "Skyline.h"
#include "frc2/command/CommandScheduler.h"
#include <frc/smartdashboard/SmartDashboard.h>

Skyline::Skyline() : TimedRobot() {}

void Skyline::RobotInit() {

}


void Skyline::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Skyline::DisabledInit() {
    frc::ChassisSpeeds chassisVels;
    chassis.SetVelocity(chassisVels);
}

void Skyline::DisabledPeriodic() {

}

void Skyline::AutonomousInit() {

}

void Skyline::AutonomousPeriodic() {

}

void Skyline::TeleopInit() {

}

void Skyline::TeleopPeriodic() {
    frc::ChassisSpeeds chassisVels;
    chassisVels.vx = units::meters_per_second_t(-joy.GetRawAxis(1));
    chassisVels.omega = units::radians_per_second_t(-joy.GetRawAxis(0));
    chassis.SetVelocity(chassisVels);

    const auto pose = chassis.getPose();

    frc::SmartDashboard::PutNumber("TankChassis_clone_0/X", pose.X().to<double>());
    frc::SmartDashboard::PutNumber("TankChassis_clone_0/Y", pose.Y().to<double>());

}

void Skyline::TestPeriodic() {

}