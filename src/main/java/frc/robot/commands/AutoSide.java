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
  public AutoSide(DriveSubsystem driveSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, AnglerSubsystem anglerSubsystem, CollectorSubsystem collectorSubsystem) {
    addCommands(new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_DIRECT));
    addCommands(shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem));
    addCommands(new SetAnglerCommand(anglerSubsystem, 0));
    // if (DriverStation.getAlliance().get() == Alliance.Blue) {
    //   addCommands(new DriveStraightenCommand(driveSubsystem, 90).withTimeout(1.5));
    // } else {
    //   addCommands(new DriveStraightenCommand(driveSubsystem, -30).withTimeout(1.5));
    // } // this doesn't work ;-;
    addCommands(new DriveForwardCommand(driveSubsystem).withTimeout(1.6));
    addCommands(new DriveStraightenCommand(driveSubsystem, 0).withTimeout(2));
    addCommands(new DriveForwardCommand(driveSubsystem).withTimeout(2.5));
  }
}
