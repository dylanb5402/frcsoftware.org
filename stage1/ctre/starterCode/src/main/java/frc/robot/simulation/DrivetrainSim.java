package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class DrivetrainSim {

    private final TalonFX leftFx, rightFx;
    private final Pigeon2 imu;
    private final TalonFXSimState leftSim, rightSim;
    private final Pigeon2SimState imuSim;
    
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
    private final StructPublisher<Pose2d> simPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SimPose", Pose2d.struct).publish();

    public DrivetrainSim(TalonFX leftFx, TalonFX rightFx, Pigeon2 imu) {
        this.leftFx = leftFx;
        this.rightFx = rightFx;
        this.imu = imu;

        this.leftSim = leftFx.getSimState();
        this.rightSim = rightFx.getSimState();
        this.imuSim = imu.getSimState();

    }
    
    public void init() {
        /*
         * Tell each simulated TalonFX which direction is "positive" so that the
         * simulated encoder counts match the physical motor orientation set above.
         */
        leftSim.Orientation = ChassisReference.CounterClockwise_Positive;
        rightSim.Orientation = ChassisReference.Clockwise_Positive;
    }

    public void periodic() {
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
        simPosePublisher.set(m_driveSim.getPose());

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
