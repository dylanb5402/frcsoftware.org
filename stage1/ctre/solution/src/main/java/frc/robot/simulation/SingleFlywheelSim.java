package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SingleFlywheelSim {

    private final TalonFX motor;
    private final TalonFXSimState motorSim;

    private final FlywheelSim m_flywheelSim;
    private double angularPositionRad = 0.0;

    private final DoublePublisher motorVoltagePub;
    private final DoublePublisher rotorVelocityPub;
    private final DoublePublisher currentPub;
    private final DoublePublisher rotorPositionPub;

    private final String name;

    public SingleFlywheelSim(TalonFX motor, String name) {
        this.name = name;
        // this(motor, DCMotor.getKrakenX60Foc(1), 1.0, 0.001);
        this.motor = motor;
        this.motorSim = motor.getSimState();
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this.m_flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(gearbox, 0.001, 1.0),
                gearbox);

        var table = NetworkTableInstance.getDefault()
                .getTable(this.name);
        this.motorVoltagePub = table.getDoubleTopic("MotorVoltage").publish();
        this.rotorVelocityPub = table.getDoubleTopic("RotorVelocity").publish();
        this.currentPub = table.getDoubleTopic("Current").publish();
        this.rotorPositionPub = table.getDoubleTopic("RotorPosition").publish();
    }

    public void init() {
        motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    public void periodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        m_flywheelSim.setInputVoltage(motorSim.getMotorVoltage());
        m_flywheelSim.update(0.02);

        double velocityRadPerSec = m_flywheelSim.getAngularVelocityRadPerSec();
        angularPositionRad += velocityRadPerSec * 0.02;

        motorSim.setRawRotorPosition(Radians.of(angularPositionRad));
        motorSim.setRotorVelocity(RadiansPerSecond.of(velocityRadPerSec));

        motorVoltagePub.set(motorSim.getMotorVoltage());
        rotorVelocityPub.set(velocityRadPerSec);
        currentPub.set(m_flywheelSim.getCurrentDrawAmps());
        rotorPositionPub.set(angularPositionRad);
    }
}
