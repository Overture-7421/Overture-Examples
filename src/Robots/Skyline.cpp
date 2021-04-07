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

void Skyline::DisabledPeriodic() {}

void Skyline::AutonomousInit() {

}

void Skyline::AutonomousPeriodic() {


}

void Skyline::TeleopInit() {

}

void Skyline::TeleopPeriodic() {
    frc::ChassisSpeeds chassisVels;
    chassisVels.vx = 1_mps;
    chassisVels.omega = 0.5_rad_per_s;
    chassis.SetVelocity(chassisVels);
    frc::SmartDashboard::PutNumber("Heading", chassis.getHeading().Degrees().to<double>());
}

void Skyline::TestPeriodic() {}
