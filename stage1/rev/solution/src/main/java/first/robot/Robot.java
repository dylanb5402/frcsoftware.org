// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.drive.DifferentialDrive;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import first.robot.simulation.DrivetrainSim;
import first.robot.simulation.SingleFlywheelSim;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * The methods in this class are called automatically as described in the OpModeRobot documentation.
 * OpMode classes anywhere in the package (or sub-packages) where this class is located are
 * automatically registered to display in the Driver Station. If you change the name of this class
 * or the package after creating this project, you must also update the Main.java file in the
 * project.
 */
public class Robot extends OpModeRobot {

  private SparkMax leftLeader = new SparkMax(0, 0, MotorType.kBrushless);
  private SparkMax leftFollower = new SparkMax(0, 1, MotorType.kBrushless);
  private SparkMax rightLeader = new SparkMax(0, 2, MotorType.kBrushless);
  private SparkMax rightFollower = new SparkMax(0, 3, MotorType.kBrushless);
  private SparkMax intake = new SparkMax(0, 4, MotorType.kBrushless);
  private SparkMax shooter = new SparkMax(0, 5, MotorType.kBrushless);
  private OnboardIMU imu;

  private DrivetrainSim drivetrainSim = new DrivetrainSim(leftLeader, rightLeader);
  private SingleFlywheelSim intakeSim = new SingleFlywheelSim(intake, "Intake");
  private SingleFlywheelSim shooterSim = new SingleFlywheelSim(shooter, "Shooter");

  public final DifferentialDrive drivetrain = new DifferentialDrive(leftLeader::setThrottle, rightLeader::setThrottle);



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    var leftConfig = new SparkMaxConfig().inverted(true);
    leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftConfig.follow(leftLeader), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var rightConfig = new SparkMaxConfig().inverted(false);
    rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightConfig.follow(rightLeader), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    imu = new OnboardIMU(MountOrientation.FLAT);
  }


  @Override
  public void simulationInit() {
    drivetrainSim.init();
    intakeSim.init();
    shooterSim.init();
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.periodic();
    intakeSim.periodic();
    shooterSim.periodic();
  }
}
