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
import frc.robot.commands.FeedSlowCommand;
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

    // Encoder object created to display position values
    m_encoder = mShooter.getEncoder();

    // PID coefficients
    //  kP = 5e-4; 
    //  kI = 0;
    //  kD = 1; 
    //  kIz = 0; 
    //  kFF = 0.000015; 
     kP = 0.005; 
     kI = 0;
     kD = 0.0001; 
     kIz = 0; 
     kFF = 0.00022; 
     kMaxOutput = 1; 
     kMinOutput = -1;
     maxRPM = 5600;
 
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
  // PID used to see if it would make any significant difference, didn't really.
  public void shootPID(double speed) {
    
    double PIDSpeed = speed*maxRPM;
    m_pidController.setReference(PIDSpeed, CANSparkMax.ControlType.kVelocity);

  }
  public void shoot() {
    mShooter.set(MotorSpeeds.SHOOTER_SPEED);
  }
  public void turnOffShooter() {
    mShooter.set(0);
  }
  // I don't know why I put this as part of the shooter subsystem, it isn't really making use of it lol, TODO: Move this to a dif file.
  public Command shootSequence(FeederSubsystem m_FeederSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    return new SequentialCommandGroup(
      // the relics of wasted time and effort
      // new ParallelCommandGroup(
      //   new ShooterCommand(m_ShooterSubsystem).withTimeout(5),
      //   new SequentialCommandGroup(
      //     new WaitCommand(4),
      //     new FeederCommand(m_FeederSubsystem, true).withTimeout(1)
      //   )
      // )
      // new ShooterPIDCommand(m_ShooterSubsystem).withTimeout(0.2),
      // new ParallelCommandGroup(
      //   new ShooterCommand(m_ShooterSubsystem).withTimeout(0.7),
        
      //   new SequentialCommandGroup(
         

      //     new FeederCommand(m_FeederSubsystem, true).withTimeout(0.4)
      //   )
      // )
      new ParallelCommandGroup(
        new ShooterCommand(m_ShooterSubsystem).withTimeout(2),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new FeedSlowCommand(m_FeederSubsystem, true).withTimeout(1)
        )
      )
      // new ShooterPIDCommand(m_ShooterSubsystem)

    );
  }
  @Override
  public void periodic() {

    // SmartDashboard.putNumber("RPM", m_encoder.getVelocity());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
