// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import first.robot.Robot;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class MyTeleop extends PeriodicOpMode {
  private final Robot robot;
  private final NiDsXboxController xboxController = new NiDsXboxController(0);

  public MyTeleop(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void periodic() {
    robot.drivetrain.arcadeDrive(-xboxController.getLeftY(), xboxController.getRightX());

    if (xboxController.getLeftBumperButton()) {
      robot.intake.setThrottle(1.0);
    } else {
      robot.intake.setThrottle(0.0);
    }

    if (xboxController.getRightBumperButton()) {
      robot.shooter.setThrottle(1.0);
    } else {
      robot.shooter.setThrottle(0.0);
    }
  }
}
