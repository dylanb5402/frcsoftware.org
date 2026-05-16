// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.simulation.DrivetrainSim;
import frc.robot.simulation.SingleFlywheelSim;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final XboxController joystick = new XboxController(0);

    private final TalonFX leftFX = new TalonFX(1);
    private final TalonFX rightFX = new TalonFX(2);
    private final TalonFX intake = new TalonFX(3);
    private final TalonFX shooter = new TalonFX(4);
    private final Pigeon2 imu = new Pigeon2(0);
    private final DifferentialDrive drivetrain = new DifferentialDrive(leftFX::set, rightFX::set);

    private DrivetrainSim sim = new DrivetrainSim(leftFX, rightFX, imu);
    private SingleFlywheelSim intakeSim = new SingleFlywheelSim(intake);
    private SingleFlywheelSim shooterSim = new SingleFlywheelSim(shooter);
    
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public Robot() {
        // Configure the left motor to run counter-clockwise when given a positive
        // output.
        TalonFXConfiguration fxCfg = new TalonFXConfiguration();
        fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftFX.getConfigurator().apply(fxCfg);

        // Configure the right motor to run clockwise when given a positive output,
        // since it is physically mirrored from the left side.
        fxCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightFX.getConfigurator().apply(fxCfg);

        Pigeon2Configuration imuCfg = new Pigeon2Configuration();
        imu.getConfigurator().apply(imuCfg);
    }

    @Override
    public void teleopPeriodic() {
        drivetrain.arcadeDrive(-joystick.getLeftY(), -joystick.getRightX());
        if (joystick.getLeftBumperButton()) {
            intake.set(0.5);
        } else {
            intake.set(0.0);
        }

        if (joystick.getRightBumperButton()) {
            shooter.set(0.5);
        } else {
            shooter.set(0.0);
        }
    }

    @Override
    public void simulationInit() {
      sim.init();
      intakeSim.init();
      shooterSim.init();
    }

    @Override
    public void simulationPeriodic() {
       sim.periodic();
       intakeSim.periodic();
       shooterSim.periodic();
    }
}