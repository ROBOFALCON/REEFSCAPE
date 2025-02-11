// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;  
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
/**
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  private final SparkMax m_leftFrontMotor = new SparkMax(5,MotorType.kBrushed);
  private final SparkMax m_leftRearMotor = new SparkMax(6,MotorType.kBrushed);
  
  private final SparkMax m_rightFrontMotor = new SparkMax(7, MotorType.kBrushed);
  private final SparkMax m_rightRearMotor = new SparkMax(8, MotorType.kBrushed);

 
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftFrontMotor :: set, m_rightFrontMotor::set);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
 

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftRearMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightRearMotor);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig m_rightFrontMotorConfig = new SparkMaxConfig();
    SparkMaxConfig m_leftRearMotorConfig = new SparkMaxConfig();
    SparkMaxConfig m_rightRearMotorConfig = new SparkMaxConfig();

    globalConfig
    .smartCurrentLimit(50)
    .idleMode(IdleMode.kBrake);

// Apply the global config and invert since it is on the opposite side
m_rightFrontMotorConfig
    .apply(globalConfig)
    .inverted(true);

// Apply the global config and set the leader SPARK for follower mode
m_leftRearMotorConfig
    .apply(globalConfig)
    .follow(m_leftFrontMotor);

// Apply the global config and set the leader SPARK for follower mode
m_rightRearMotorConfig
    .apply(globalConfig)
    .follow(m_rightFrontMotor);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightFrontMotor.setInverted(true);
    m_leftFrontMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftRearMotor.configure(m_leftRearMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFrontMotor.configure(m_rightFrontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightRearMotor.configure(m_rightRearMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    double forward = -m_controller.getLeftY();
    double rotation = m_controller.getRightX();

    /*
     * Apply values to left and right side. We will only need to set the leaders
     * since the other motors are in follower mode.
     */
    m_leftFrontMotor.set(forward + rotation);
    m_rightFrontMotor.set(forward - rotation);
  //  m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
