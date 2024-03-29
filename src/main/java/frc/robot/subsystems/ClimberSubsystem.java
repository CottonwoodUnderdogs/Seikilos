// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    mClimberLeft.restoreFactoryDefaults();
    mClimberRight.restoreFactoryDefaults();

    mClimberLeft.setIdleMode(IdleMode.kBrake);
    mClimberRight.setIdleMode(IdleMode.kBrake);

    mClimberLeft.setSmartCurrentLimit(38);
    mClimberRight.setSmartCurrentLimit(38);
    mClimberRight.setSecondaryCurrentLimit(40);
    mClimberLeft.setSecondaryCurrentLimit(40);

    m_leftPidController = mClimberLeft.getPIDController();
    m_rightPidController = mClimberRight.getPIDController();

    // Encoder object created to display position values
    m_leftEncoder = mClimberLeft.getEncoder();
    m_rightEncoder = mClimberRight.getEncoder();

    // PID coefficients
    kPRight = 0.05; 
    kPLeft = 0.10;
    kI = 0;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_leftPidController.setP(kPLeft);
    m_leftPidController.setI(kI);
    m_leftPidController.setD(kD);
    m_leftPidController.setIZone(kIz);
    m_leftPidController.setFF(kFF);
    m_leftPidController.setOutputRange(kMinOutput, kMaxOutput);

    m_rightPidController.setP(kPRight);
    m_rightPidController.setI(kI);
    m_rightPidController.setD(kD);
    m_rightPidController.setIZone(kIz);
    m_rightPidController.setFF(kFF);
    m_rightPidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  // motor controller setup
  CANSparkMax mClimberLeft = new CANSparkMax(Constants.MotorID.CLIMBER_LEFT, MotorType.kBrushless);
  CANSparkMax mClimberRight  = new CANSparkMax(Constants.MotorID.CLIMBER_RIGHT, MotorType.kBrushless);

  DigitalInput climberLeftLimit = new DigitalInput(Constants.Inputs.LEFT_CLIMBER_CHANNEL);
  DigitalInput climberRightLimit = new DigitalInput(Constants.Inputs.RIGHT_CLIMBER_CHANNEL);
  
  private SparkPIDController m_leftPidController;
  private RelativeEncoder m_leftEncoder;
  public double kPLeft, kPRight, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public static double rotationsLeft = 0;
  public static double rotationsRight = 0;

  private SparkPIDController m_rightPidController;
  private RelativeEncoder m_rightEncoder;


  /**
   * Example command factory method.
   *
   * @return a command
   */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }
  public double getLeftPosition() {
    return rotationsLeft;
  }
  public double getRightPosition() {
    return rotationsRight;
  }

  /**
   * Dangerous, if limit switches are not working or the climbers go too high, there is nothing to stop this from breaking the climbers, use setClimberRotations
   *
   */
  public void climbLeft(double speed) {
    if (climberLeftLimit.get()) {
      mClimberLeft.set(speed);
    } else {
      mClimberLeft.set(0);
      m_leftEncoder.setPosition(0);
    }
    
  }
  /**
   * Dangerous, if limit switches are not working or the climbers go too high, there is nothing to stop this from breaking the climbers, use setClimberRotations
   *
   */
  public void climbRight(double speed) {
    if (climberRightLimit.get()) {
      mClimberRight.set(speed);
    } else {
      mClimberRight.set(0);
      m_rightEncoder.setPosition(0);
    } 

  }
  /**
   * uses PID to control height of climbers, with PID limits.
   * @param rotations
   */
  public void setClimberLeftRotations(double rotations) {
    rotationsLeft -= rotations;
    if (rotationsLeft >= -230 && rotationsLeft <= 0) {
      m_leftPidController.setReference(rotationsLeft, CANSparkMax.ControlType.kPosition);
    } else if (rotationsLeft < -230) {
      rotationsLeft = -230;
    } else if (rotationsLeft > 0) {
      rotationsLeft = 0;
    }
  }
  /**
   * uses PID to control height of climbers, with PID limits.
   * @param rotations
   */
  public void setClimberRightRotations(double rotations) {
    rotationsRight += rotations;

    if (rotationsRight <= 230 && rotationsRight >= 0) {
      m_rightPidController.setReference(rotationsRight, CANSparkMax.ControlType.kPosition);
    } else if (rotationsRight > 230) {
      rotationsRight = 230;
    } else if (rotationsRight < 0) {
      rotationsRight = 0;
    }
    
  }
  @Override
  public void periodic() {
    
  } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
