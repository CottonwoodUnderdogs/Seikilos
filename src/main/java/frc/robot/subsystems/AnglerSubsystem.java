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

public class AnglerSubsystem extends SubsystemBase {
  /** Creates a new AnglerSubsystem. */
  public AnglerSubsystem() {
    mAngler.restoreFactoryDefaults();
    mAngler.setIdleMode(IdleMode.kBrake);

    mAngler.setSmartCurrentLimit(38);
    mAngler.setSecondaryCurrentLimit(40);
  }

  CANSparkMax mAngler = new CANSparkMax(MotorID.ANGLER, MotorType.kBrushless);

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

  public void anglePower(double speed) {
    mAngler.set(speed);
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
