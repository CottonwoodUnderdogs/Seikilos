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
public class AutoStraight extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */
  public AutoStraight(DriveSubsystem driveSubsystem, FeederSubsystem feederSubsystem, AnglerSubsystem anglerSubsystem, CollectorSubsystem collectorSubsystem) {
    addCommands(new WaitCommand(13.5));
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_COLLECTING),
        new FeederCommand(feederSubsystem, false).deadlineWith(new CollectorCommand(collectorSubsystem)),
        new DriveForwardCommand(driveSubsystem).withTimeout(0.7)
      )
    );  
  }
}
