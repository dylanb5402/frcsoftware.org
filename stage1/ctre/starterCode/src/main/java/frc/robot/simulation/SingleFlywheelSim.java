package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SingleFlywheelSim {

    private final TalonFX motor;
    private final TalonFXSimState motorSim;

    private final FlywheelSim m_flywheelSim;
    private double angularPositionRad = 0.0;

    public SingleFlywheelSim(TalonFX motor) {
        this(motor, DCMotor.getKrakenX60Foc(1), 1.0, 0.001);
    }

    public SingleFlywheelSim(TalonFX motor, DCMotor gearbox, double gearing, double moi) {
        this.motor = motor;
        this.motorSim = motor.getSimState();
        this.m_flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(gearbox, moi, gearing),
                gearbox);
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
    }
}
