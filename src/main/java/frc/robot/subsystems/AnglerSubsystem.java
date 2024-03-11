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

public class AnglerSubsystem extends SubsystemBase {
  /** Creates a new AnglerSubsystem. */
  public AnglerSubsystem() {
    mAngler.restoreFactoryDefaults();
    mAngler.setIdleMode(IdleMode.kBrake);

    mAngler.setSmartCurrentLimit(38);
    mAngler.setSecondaryCurrentLimit(40);

    m_pidController = mAngler.getPIDController();
    m_encoder = mAngler.getEncoder();

    kP = 0.05; 
    kI = 0;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.8; 
    kMinOutput = -0.8;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  CANSparkMax mAngler = new CANSparkMax(MotorID.ANGLER, MotorType.kBrushless);
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public static double rotations = 0;  // 14 is perfect for close up shooting

  public void setAngle(double rotations) {
    AnglerSubsystem.rotations = rotations;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (rotations <= 0 && rotations >= -26) {
      m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    } else if (rotations > 0) {
      rotations = 0;
    } else if (rotations < -26) {
      rotations = -26;
    }
    SmartDashboard.putNumber("Rotations", rotations);
  } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
