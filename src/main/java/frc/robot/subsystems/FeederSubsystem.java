// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    mFeeder.restoreFactoryDefaults();
    mFeeder.setIdleMode(IdleMode.kBrake);

    mFeeder.setSmartCurrentLimit(38);
    mFeeder.setSecondaryCurrentLimit(40);
  }

  CANSparkMax mFeeder = new CANSparkMax(MotorID.FEEDER, MotorType.kBrushless);
  DigitalInput noteLoadedInput = new DigitalInput(Inputs.NOTE_LOADED_CHANNEL);

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean isNoteLoaded() {
    // Query some boolean state, such as a digital sensor.
    return !noteLoadedInput.get();
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
