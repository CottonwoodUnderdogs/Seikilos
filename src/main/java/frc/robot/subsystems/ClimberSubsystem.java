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
    kP = 0.05; 
    kI = 0;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
  }

  // motor controller setup
  CANSparkMax mClimberLeft = new CANSparkMax(Constants.MotorID.CLIMBER_LEFT, MotorType.kBrushless);
  CANSparkMax mClimberRight  = new CANSparkMax(Constants.MotorID.CLIMBER_RIGHT, MotorType.kBrushless);

  DigitalInput climberLeftLimit = new DigitalInput(Constants.Inputs.LEFT_CLIMBER_CHANNEL);
  DigitalInput climberRightLimit = new DigitalInput(Constants.Inputs.RIGHT_CLIMBER_CHANNEL);
  
  private SparkPIDController m_leftPidController;
  private RelativeEncoder m_leftEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double rotationsLeft = 0;
  public double rotationsRight = 0;

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

  public void climbLeft(double speed) {
    if (climberLeftLimit.get()) {
      mClimberLeft.set(speed);
    } else {
      if (speed >= 0) {
        mClimberLeft.set(0);
      } else {
        mClimberLeft.set(speed);
      }
      
    }
    
  }
  public void climbRight(double speed) {
    if (climberRightLimit.get()) {
      mClimberRight.set(speed);
    } else {
      if (speed <= 0) {
        mClimberRight.set(0);
      } else {
        mClimberRight.set(speed);
      }
    }
    
  }
  @Override
  public void periodic() {
    if (climberLeftLimit.get()) {
      m_leftPidController.setReference(rotationsLeft, CANSparkMax.ControlType.kPosition);
    } else if (rotationsLeft < -587) {
      rotationsLeft = -587;
    }
    if (climberRightLimit.get()) {
      m_rightPidController.setReference(rotationsRight, CANSparkMax.ControlType.kPosition);
    } else if (rotationsRight > 235) {
      rotationsRight = 235;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
