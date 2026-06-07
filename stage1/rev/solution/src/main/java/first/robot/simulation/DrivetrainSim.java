// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.simulation;

import com.revrobotics.spark.SparkMax;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.simulation.DifferentialDrivetrainSim;
import org.wpilib.simulation.OnboardIMUSim;

public class DrivetrainSim {

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

  private final DoublePublisher leftPositionPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("DrivetrainSim/LeftPositionMeters")
          .publish();
  private final DoublePublisher rightPositionPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("DrivetrainSim/RightPositionMeters")
          .publish();
  private final DoublePublisher leftVelocityPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("DrivetrainSim/LeftVelocityMPS")
          .publish();
  private final DoublePublisher rightVelocityPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("DrivetrainSim/RightVelocityMPS")
          .publish();

  private final DoublePublisher leftVoltagePub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("DrivetrainSim/LeftMotorVoltage")
          .publish();
  private final DoublePublisher rightVoltagePub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("DrivetrainSim/RightMotorVoltage")
          .publish();
  private final DoublePublisher leftCurrentPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("DrivetrainSim/LeftCurrentAmps")
          .publish();
  private final DoublePublisher rightCurrentPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("DrivetrainSim/RightCurrentAmps")
          .publish();

  public DrivetrainSim(SparkMax leftSpark, SparkMax rightSpark) {
    this.leftSpark = leftSpark;
    this.rightSpark = rightSpark;
  }

  public void periodic() {
    double leftMotorVoltage = leftSpark.getThrottle() * kBusVoltage;
    double rightMotorVoltage = rightSpark.getThrottle() * kBusVoltage;

    m_driveSim.setInputs(leftMotorVoltage, rightMotorVoltage);
    m_driveSim.update(0.02);

    OnboardIMUSim.setYaw(m_driveSim.getHeading().getRadians());

    simPosePublisher.set(m_driveSim.getPose());
    leftPositionPub.set(m_driveSim.getLeftPosition());
    rightPositionPub.set(m_driveSim.getRightPosition());
    leftVelocityPub.set(m_driveSim.getLeftVelocity());
    rightVelocityPub.set(m_driveSim.getRightVelocity());
    leftVoltagePub.set(leftMotorVoltage);
    rightVoltagePub.set(rightMotorVoltage);
    leftCurrentPub.set(m_driveSim.getLeftCurrentDraw());
    rightCurrentPub.set(m_driveSim.getRightCurrentDraw());
  }
}
