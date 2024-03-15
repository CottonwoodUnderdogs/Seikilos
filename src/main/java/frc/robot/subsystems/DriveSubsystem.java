// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Inputs;
import frc.robot.Constants.MotorSpeeds;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    mFrontRight.restoreFactoryDefaults();
    mFrontLeft.restoreFactoryDefaults();
    mBackRight.restoreFactoryDefaults();
    mBackLeft.restoreFactoryDefaults();

    mFrontRight.setIdleMode(IdleMode.kBrake);
    mFrontLeft.setIdleMode(IdleMode.kBrake);
    mBackRight.setIdleMode(IdleMode.kBrake);
    mBackLeft.setIdleMode(IdleMode.kBrake);

    mFrontRight.setInverted(true);
    mBackRight.setInverted(true);

    mFrontRight.setSmartCurrentLimit(38);
    mFrontLeft.setSmartCurrentLimit(38);
    mBackRight.setSmartCurrentLimit(38);
    mBackLeft.setSmartCurrentLimit(38);
    mFrontRight.setSecondaryCurrentLimit(40);
    mFrontLeft.setSecondaryCurrentLimit(40);
    mBackRight.setSecondaryCurrentLimit(40);
    mBackLeft.setSecondaryCurrentLimit(40);

    
  }

  // motor controller setup
  CANSparkMax mFrontRight = new CANSparkMax(Constants.MotorID.FRONT_RIGHT, MotorType.kBrushless);
  CANSparkMax mFrontLeft  = new CANSparkMax(Constants.MotorID.FRONT_LEFT, MotorType.kBrushless);
  CANSparkMax mBackRight  = new CANSparkMax(Constants.MotorID.BACK_RIGHT, MotorType.kBrushless);
  CANSparkMax mBackLeft   = new CANSparkMax(Constants.MotorID.BACK_LEFT, MotorType.kBrushless);

  private final MecanumDrive robotDrive = new MecanumDrive(mFrontLeft, mBackLeft, mFrontRight, mBackRight);
  private final AHRS m_gyro = new AHRS(Port.kUSB);
  private double driveSpeed = MotorSpeeds.DRIVE_SPEED;
  public static boolean isAuto;
  
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
  public void zeroGyro() {
    m_gyro.reset();
  }
  public void driveCartesian(double vx, double vy, double omega) {
    robotDrive.driveCartesian(vx * driveSpeed, vy * driveSpeed, omega * driveSpeed);
  }
  // uses gyro to make field centric driving
  public void driveFieldCartesian(double vx, double vy, double omega) {
    robotDrive.driveCartesian(vx * driveSpeed, vy * driveSpeed, omega * driveSpeed, m_gyro.getRotation2d().times(-1));
  }
  public void straighten(double desiredAngle) {
    // keep in mind that this stops all other driving, so it must be called when not driving.
    double omega = desiredAngle - m_gyro.getAngle();
    this.driveCartesian(0, 0, omega*0.01); // multiplying error by constant to correct it until it hits its target 0.
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("angle", m_gyro.getYaw());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
