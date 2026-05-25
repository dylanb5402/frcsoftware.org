// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.simulation;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.FlywheelSim;

/**
 * Simulation wrapper for a single flywheel driven by a REV SparkMax motor controller. Call {@link
 * #init()} once before enabling and {@link #periodic()} every 20ms from the robot's {@code
 * simulationPeriodic()} method.
 */
public class SingleFlywheelSim {

  private final SparkMax motor;
  private SparkMaxSim motorSim;

  private final FlywheelSim m_flywheelSim;

  private final DoublePublisher motorVoltagePub;
  private final DoublePublisher rotorVelocityPub;
  private final DoublePublisher currentPub;
  private final DoublePublisher rotorPositionPub;

  private static final double kBusVoltage = 12.0;

  private final String name;

  /**
   * Creates a flywheel simulation that bridges a REV SparkMax motor controller with WPILib's {@link
   * FlywheelSim} physics model.
   *
   * @param motor the SparkMax motor controller
   * @param name the NetworkTables table name for publishing telemetry
   */
  public SingleFlywheelSim(SparkMax motor, String name) {
    this.name = name;
    this.motor = motor;
    var gearbox = DCMotor.getNEO(1);
    this.m_flywheelSim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(gearbox, 0.001, 1.0), gearbox);

    var table = NetworkTableInstance.getDefault().getTable(this.name);
    this.motorVoltagePub = table.getDoubleTopic("MotorVoltage").publish();
    this.rotorVelocityPub = table.getDoubleTopic("RotorVelocity").publish();
    this.currentPub = table.getDoubleTopic("Current").publish();
    this.rotorPositionPub = table.getDoubleTopic("RotorPosition").publish();
  }

  /**
   * Call once after construction. SparkMax inversion/orientation is handled through motor controller
   * configuration rather than in simulation setup.
   */
  public void init() {
    this.motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
  }

  /**
   * Call every 20ms from {@code simulationPeriodic()}. Reads the applied output from the SparkMaxSim
   * (one-tick delayed from the previous iterate), feeds the resulting motor voltage into the
   * flywheel physics model, then writes the resulting velocity back to the SparkMaxSim.
   */
  public void periodic() {
    motorSim.setBusVoltage(kBusVoltage);

    double motorVoltage = motorSim.getAppliedOutput() * kBusVoltage;
    m_flywheelSim.setInputVoltage(motorVoltage);
    m_flywheelSim.update(0.02);

    double velocityRadPerSec = m_flywheelSim.getAngularVelocity();
    double velocityRPM = velocityRadPerSec * 60.0 / (2.0 * Math.PI);

    motorSim.iterate(velocityRPM, kBusVoltage, 0.02);

    double positionRad = motorSim.getPosition() * 2.0 * Math.PI;

    motorVoltagePub.set(motorVoltage);
    rotorVelocityPub.set(velocityRadPerSec);
    currentPub.set(m_flywheelSim.getCurrentDraw());
    rotorPositionPub.set(positionRad);
  }
}
