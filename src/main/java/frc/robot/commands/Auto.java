// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Auto {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DriveSubsystem subsystem) {
    // return Commands.sequence(subsystem.driveCartesian(0.2, 0, 0);, new DriveCommand(subsystem));
    return null;
  }

  private Auto() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
