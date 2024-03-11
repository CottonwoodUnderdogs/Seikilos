// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.SetPoints;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.stream.Collector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
public class AutoSide extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */
  Optional<Alliance> ally = DriverStation.getAlliance();
  boolean isBlue = ally.get() == Alliance.Blue;
  public AutoSide(DriveSubsystem driveSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, AnglerSubsystem anglerSubsystem) {
    addCommands(new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_DIRECT));
    addCommands(shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem));
    addCommands(new DriveStraightenCommand(driveSubsystem, 0).withTimeout(0.25));
    addCommands(new SetAnglerCommand(anglerSubsystem, 0));
    if (isBlue) {
      addCommands(new DriveRightCommand(driveSubsystem).withTimeout(4));
    } else {
      addCommands(new DriveLeftCommand(driveSubsystem).withTimeout(4));
    }
    addCommands(new DriveForwardCommand(driveSubsystem).withTimeout(2));
  }
}
