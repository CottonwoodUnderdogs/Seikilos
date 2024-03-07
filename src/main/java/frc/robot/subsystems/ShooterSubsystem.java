// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterPIDCommand;

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

    mShooter2.restoreFactoryDefaults();
    mShooter2.setIdleMode(IdleMode.kCoast);

    mShooter2.setSmartCurrentLimit(40);
    mShooter2.setSecondaryCurrentLimit(50);

    m_pidController2 = mShooter2.getPIDController();

    // Encoder object created to display position values
    m_encoder2 = mShooter2.getEncoder();

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

     m_pidController2.setP(kP);
     m_pidController2.setI(kI);
     m_pidController2.setD(kD);
     m_pidController2.setIZone(kIz);
     m_pidController2.setFF(kFF);
     m_pidController2.setOutputRange(kMinOutput, kMaxOutput);

     

  }

  CANSparkMax mShooter = new CANSparkMax(MotorID.SHOOTER, MotorType.kBrushless);
  CANSparkMax mShooter2 = new CANSparkMax(MotorID.SHOOTER2, MotorType.kBrushless);
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_pidController2;
  private RelativeEncoder m_encoder2;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  double setPoint = 0.8*maxRPM;
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

  public void shootPID(double speed) {
    
    double PIDSpeed = speed*maxRPM;
    m_pidController.setReference(PIDSpeed, CANSparkMax.ControlType.kVelocity);
    m_pidController2.setReference(PIDSpeed, CANSparkMax.ControlType.kVelocity);

  }
  public void shoot(double speed) {
    mShooter.set(speed);
    mShooter2.set(speed);
  }
  public Command shootSequence(FeederSubsystem m_FeederSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ShooterPIDCommand(m_ShooterSubsystem).withTimeout(0.5),
        new ShooterCommand(m_ShooterSubsystem).withTimeout(2),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new FeederCommand(m_FeederSubsystem, true).withTimeout(5)
        )
      )
    );
  }

  @Override
  public void periodic() {

    
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
