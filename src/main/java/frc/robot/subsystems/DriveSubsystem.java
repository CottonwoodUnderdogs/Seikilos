// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    mFrontRight.restoreFactoryDefaults();
    mFrontLeft.restoreFactoryDefaults();
    mBackRight.restoreFactoryDefaults();
    mBackLeft.restoreFactoryDefaults();

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
  CANSparkMax mFrontLeft  = new CANSparkMax(Constants.MotorID.FRONT_RIGHT, MotorType.kBrushless);
  CANSparkMax mBackRight  = new CANSparkMax(Constants.MotorID.FRONT_RIGHT, MotorType.kBrushless);
  CANSparkMax mBackLeft   = new CANSparkMax(Constants.MotorID.FRONT_RIGHT, MotorType.kBrushless);

  private final MecanumDrive robotDrive = new MecanumDrive(mFrontLeft, mBackLeft, mFrontRight, mBackRight);

  
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

  public void driveCartesian(double vx, double vy, double omega) {
    // TODO: try squaring inputs, possible solution instead of implementing slow mode
    robotDrive.driveCartesian(vx * 0.8, vy * 0.8, omega * 0.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
