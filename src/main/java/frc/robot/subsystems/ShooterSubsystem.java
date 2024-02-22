// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    mShooter.restoreFactoryDefaults();
    mShooter.setIdleMode(IdleMode.kCoast);

    mShooter.setSmartCurrentLimit(38);
    mShooter.setSecondaryCurrentLimit(40);
  }

  CANSparkMax mShooter = new CANSparkMax(MotorID.SHOOTER, MotorType.kBrushless);
  CANSparkMax mFeeder  = new CANSparkMax(MotorID.FEEDER, MotorType.kBrushless);

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
  public void feed(double speed) {
    mFeeder.set(speed);
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
