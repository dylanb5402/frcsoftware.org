// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // private DrivetrainSim sim = new DrivetrainSim(leftFX, rightFX, imu);
  // private SingleFlywheelSim intakeSim = new SingleFlywheelSim(intake, "Intake");
  // private SingleFlywheelSim shooterSim = new SingleFlywheelSim(shooter, "Shooter");

  public Robot() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void simulationInit() {
    // sim.init();
    // intakeSim.init();
    // shooterSim.init();
  }

  @Override
  public void simulationPeriodic() {
    // sim.periodic();
    // intakeSim.periodic();
    // shooterSim.periodic();
  }
}
