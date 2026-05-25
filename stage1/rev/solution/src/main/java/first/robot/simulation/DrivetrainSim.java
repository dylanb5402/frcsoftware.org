// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.simulation;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.simulation.DifferentialDrivetrainSim;
import org.wpilib.simulation.OnboardIMUSim;

/**
 * Simulation wrapper for a differential drivetrain using REV SparkMax motor controllers and the
 * Systemcore onboard IMU. Call {@link #init()} once before enabling and {@link #periodic()} every
 * 20ms from the robot's {@code simulationPeriodic()} method.
 */
public class DrivetrainSim {

  private SparkMaxSim leftSim, rightSim;
  private final SparkMax leftSpark, rightSpark;

  private final double kGearRatio = 10.71;
  private final double kWheelRadiusMeters = 0.0762; // 3 inches
  private static final double kBusVoltage = 12.0;

  private final DifferentialDrivetrainSim m_driveSim =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
          kGearRatio,
          2.1, // MOI of 2.1 kg m^2 (from CAD model).
          26.5, // Mass of the robot is 26.5 kg.
          kWheelRadiusMeters, // Robot uses 3" radius (6" diameter) wheels.
          0.546, // Distance between wheels in meters.
          null);

  private final StructPublisher<Pose2d> simPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("SimPose", Pose2d.struct)
          .publish();

  /**
   * Creates a drivetrain simulation that bridges the REV SparkMax motor controllers and the
   * Systemcore onboard IMU with WPILib's {@link DifferentialDrivetrainSim} physics model.
   *
   * @param leftSpark the left-side SparkMax motor controller
   * @param rightSpark the right-side SparkMax motor controller
   * @param imu the Systemcore onboard IMU
   */
  public DrivetrainSim(SparkMax leftSpark, SparkMax rightSpark) {
    this.leftSpark = leftSpark;
    this.rightSpark = rightSpark;
  }

  /**
   * Call once after construction. SparkMax inversion/orientation is handled through motor controller
   * configuration (e.g. {@link com.revrobotics.spark.config.SparkBaseConfig#inverted}) rather than
   * in simulation setup.
   */
  public void init() {
    this.leftSim = new SparkMaxSim(leftSpark, DCMotor.getNEO(2));
    this.rightSim = new SparkMaxSim(rightSpark, DCMotor.getNEO(2));
  }

  /**
   * Call every 20ms from {@code simulationPeriodic()}. Reads the applied output from each
   * SparkMaxSim (one-tick delayed from the previous iterate), feeds the resulting motor voltage into
   * the physics model, then writes the resulting velocity and position back to the SparkMaxSim and
   * the heading to the OnboardIMU simulation.
   */
  public void periodic() {
    leftSim.setBusVoltage(kBusVoltage);
    rightSim.setBusVoltage(kBusVoltage);

    double leftMotorVoltage = leftSim.getAppliedOutput() * kBusVoltage;
    double rightMotorVoltage = rightSim.getAppliedOutput() * kBusVoltage;

    m_driveSim.setInputs(leftMotorVoltage, rightMotorVoltage);
    m_driveSim.update(0.02);

    double leftRPM = metersPerSecToRPM(m_driveSim.getLeftVelocity());
    double rightRPM = metersPerSecToRPM(m_driveSim.getRightVelocity());

    leftSim.iterate(leftRPM, kBusVoltage, 0.02);
    rightSim.iterate(rightRPM, kBusVoltage, 0.02);

    OnboardIMUSim.setYaw(m_driveSim.getHeading().getRadians());

    simPosePublisher.set(m_driveSim.getPose());
  }

  /**
   * Converts a linear wheel velocity (m/s) into motor RPM. Uses the same wheel radius and gearing
   * ratio as the physics simulation. The SparkMax relative encoder defaults to a velocity
   * conversion factor of 1 (RPM); if a custom factor is configured, this method must be updated
   * accordingly.
   */
  private double metersPerSecToRPM(double metersPerSecond) {
    return metersPerSecond / kWheelRadiusMeters * kGearRatio * 60.0 / (2.0 * Math.PI);
  }
}
