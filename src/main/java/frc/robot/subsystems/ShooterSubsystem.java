// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    mShooter.restoreFactoryDefaults();
    mShooter.setIdleMode(IdleMode.kCoast);

    mShooter.setSmartCurrentLimit(40);
    mShooter.setSecondaryCurrentLimit(50);

    m_pidController = mShooter.getPIDController();

    // Encoder object created to display position values
    m_encoder = mShooter.getEncoder();

     // PID coefficients
     kP = 5e-4; 
     kI = 0;
     kD = 1; 
     kIz = 0; 
     kFF = 0.000015; 
     kMaxOutput = 1; 
     kMinOutput = -1;
     maxRPM = 5700;
 
     // set PID coefficients
     m_pidController.setP(kP);
     m_pidController.setI(kI);
     m_pidController.setD(kD);
     m_pidController.setIZone(kIz);
     m_pidController.setFF(kFF);
     m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  }

  CANSparkMax mShooter = new CANSparkMax(MotorID.SHOOTER, MotorType.kBrushless);
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

  public void shoot(double speed) {
    mShooter.set(speed);
  }
  public void shootPID(double speed) {
    double PIDSpeed = 1.0*maxRPM;
    m_pidController.setReference(PIDSpeed, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    
    // SmartDashboard.putNumber("SetPoint", setPoint);
    // SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
