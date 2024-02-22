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
  }

  // motor controller setup
  CANSparkMax mClimberLeft = new CANSparkMax(Constants.MotorID.CLIMBER_LEFT, MotorType.kBrushless);
  CANSparkMax mClimberRight  = new CANSparkMax(Constants.MotorID.CLIMBER_RIGHT, MotorType.kBrushless);

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
    mClimberLeft.set(speed);
  }
  public void climbRight(double speed) {
    mClimberRight.set(speed);
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
