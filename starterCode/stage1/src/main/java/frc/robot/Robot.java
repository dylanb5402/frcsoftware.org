// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

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
    private final Pigeon2 imu = new Pigeon2(0);

    private final TalonFXSimState leftSim = leftFX.getSimState();
    private final TalonFXSimState rightSim = rightFX.getSimState();
    private final Pigeon2SimState imuSim = imu.getSimState();

    private final DifferentialDrive drivetrain = new DifferentialDrive(leftFX::set, rightFX::set);
    private final StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    private final double kGearRatio = 10.71;
    private final Distance kWheelRadius = Inches.of(3);

    /* Simulation model of the drivetrain */
    private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getKrakenX60Foc(2), // 2 Kraken X60 on each side of the drivetrain.
            kGearRatio,                 // Standard AndyMark gearing reduction.
            2.1,                        // MOI of 2.1 kg m^2 (from CAD model).
            26.5,                       // Mass of the robot is 26.5 kg.
            kWheelRadius.in(Meters),    // Robot uses 3" radius (6" diameter) wheels.
            0.546,                      // Distance between wheels in meters.
            null);

    /*
     * Creating my odometry object. The starting position and heading of the robot
     * on the field are both zero here, meaning we start at the field origin facing
     * the positive X direction.
     */
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            imu.getRotation2d(),
            0, 0);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public Robot() {
        StatusCode returnCode;

        // Configure the left motor to run counter-clockwise when given a positive
        // output. Retry up to 5 times in case of a CAN bus error on startup.
        TalonFXConfiguration fxCfg = new TalonFXConfiguration();
        fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        for (int i = 0; i < 5; ++i) {
            returnCode = leftFX.getConfigurator().apply(fxCfg);
            if (returnCode.isOK()) break;
        }

        // Configure the right motor to run clockwise when given a positive output,
        // since it is physically mirrored from the left side.
        fxCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        for (int i = 0; i < 5; ++i) {
            returnCode = rightFX.getConfigurator().apply(fxCfg);
            if (returnCode.isOK()) break;
        }

        Pigeon2Configuration imuCfg = new Pigeon2Configuration();
        imu.getConfigurator().apply(imuCfg);

        /*
         * Speed up the position and heading signals to 100 Hz so that odometry
         * updates are as fresh as possible. When all devices share a CANivore bus,
         * signals set to the same frequency are automatically time-synchronized.
         */
        BaseStatusSignal.setUpdateFrequencyForAll(100,
                leftFX.getPosition(),
                rightFX.getPosition(),
                imu.getYaw());
    }

    @Override
    public void robotPeriodic() {
        /*
         * Update odometry with the latest IMU heading and wheel positions.
         * getPosition() returns rotor rotations, so rotationsToMeters() accounts
         * for the gear ratio to give us the actual distance each wheel has traveled.
         */
        m_odometry.update(
                imu.getRotation2d(),
                rotationsToMeters(leftFX.getPosition().getValue()).in(Meters),
                rotationsToMeters(rightFX.getPosition().getValue()).in(Meters));

        publisher.set(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationInit() {
        /*
         * Tell each simulated TalonFX which direction is "positive" so that the
         * simulated encoder counts match the physical motor orientation set above.
         */
        leftSim.Orientation = ChassisReference.CounterClockwise_Positive;
        rightSim.Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void simulationPeriodic() {
        /*
         * Simulate supply voltage for each device. In a real robot this comes from
         * the battery; here we use WPILib's simulated battery voltage.
         */
        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        /*
         * Feed the voltage each motor is currently outputting into the drivetrain
         * physics simulation, then step the simulation forward by one robot loop (20ms).
         */
        m_driveSim.setInputs(
                leftSim.getMotorVoltage(),
                rightSim.getMotorVoltage());
        m_driveSim.update(0.02);

        /*
         * Push the simulated wheel positions and velocities back into the TalonFX
         * sim states so that getPosition() and getVelocity() return realistic values.
         * metersToRotations() converts wheel-shaft distance back to rotor rotations
         * (multiplying by the gear ratio internally) to match what the real encoder
         * would report.
         */
        leftSim.setRawRotorPosition(
                metersToRotations(Meters.of(m_driveSim.getLeftPositionMeters())));
        leftSim.setRotorVelocity(
                metersToRotationsVel(MetersPerSecond.of(m_driveSim.getLeftVelocityMetersPerSecond())));
        rightSim.setRawRotorPosition(
                metersToRotations(Meters.of(m_driveSim.getRightPositionMeters())));
        rightSim.setRotorVelocity(
                metersToRotationsVel(MetersPerSecond.of(m_driveSim.getRightVelocityMetersPerSecond())));

        imuSim.setRawYaw(m_driveSim.getHeading().getDegrees());
    }

    @Override
    public void teleopPeriodic() {
        /*
         * Arcade drive: the left stick Y axis controls forward/backward speed and
         * the right stick X axis controls turning. Both axes are negated because
         * pushing the stick forward produces a negative value by default.
         */
        drivetrain.arcadeDrive(-joystick.getLeftY(), -joystick.getRightX());
    }

    /**
     * Converts a rotor rotation measurement into linear wheel distance.
     * Because the TalonFX encoder sits before the gearbox, we first divide
     * by the gear ratio to get wheel-shaft rotations, then multiply by the
     * wheel circumference (radius × radians).
     */
    private Distance rotationsToMeters(Angle rotations) {
        var gearedRadians = rotations.in(Radians) / this.kGearRatio;
        return this.kWheelRadius.times(gearedRadians);
    }

    /**
     * Converts a linear wheel distance into rotor rotations.
     * This is the inverse of rotationsToMeters() and is used to feed
     * simulated wheel positions back into the TalonFX sim state.
     */
    private Angle metersToRotations(Distance meters) {
        var wheelRadians = meters.in(Meters) / this.kWheelRadius.in(Meters);
        return Radians.of(wheelRadians * this.kGearRatio);
    }

    /**
     * Converts a linear wheel velocity into rotor angular velocity.
     * Mirrors the logic of metersToRotations() but operates on velocity
     * rather than position.
     */
    private AngularVelocity metersToRotationsVel(LinearVelocity meters) {
        var wheelRadians = meters.in(MetersPerSecond) / this.kWheelRadius.in(Meters);
        return RadiansPerSecond.of(wheelRadians * this.kGearRatio);
    }
}