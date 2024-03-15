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

import java.util.function.BooleanSupplier;
import java.util.stream.Collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoDirect extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */
  public AutoDirect(DriveSubsystem driveSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, AnglerSubsystem anglerSubsystem, CollectorSubsystem collectorSubsystem) {
    // return Commands.sequence(subsystem.driveCartesian(0.2, 0, 0);, new DriveCommand(subsystem));
    // addCommands(
      // Commands.print("Angling"),
      // new AnglerPresetDirectCommand(anglerSubsystem),
      // Commands.print("ParallelGroup"),
      // new ParallelCommandGroup(
      //   Commands.print("Shooting"),
      //   new ShooterCommand(shooterSubsystem).withTimeout(2), // the time outs in this sequence are just for turning off the motors after a bit.
      //   new SequentialCommandGroup(
      //     new WaitCommand(0.9),
      //     new FeederCommand(feederSubsystem, true).withTimeout(1)
      //   ) 
      // ),
    this.setName("direct");
    // addCommands(new AnglerPresetDirectCommand(anglerSubsystem).withTimeout(1));
    // pre loaded note
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_DIRECT),
        shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem),
        new DriveStraightenCommand(driveSubsystem, 0).withTimeout(0.3)
      )
    );
    // second note
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_COLLECTING),
        new CollectorCommand(collectorSubsystem).withTimeout(2.5),
        new FeederCommand(feederSubsystem, false),
        new DriveForwardCommand(driveSubsystem).withTimeout(1.2)
      )
    );
    addCommands(
      new ParallelCommandGroup(
        new DriveStraightenCommand(driveSubsystem, 0).withTimeout(0.2),
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_SMALL_DISTANCE)
      )
    );
    addCommands(
      new ParallelCommandGroup(
        new DriveBackwardCommand(driveSubsystem).withTimeout(0.45),
        shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem)
      )
    );
    // third note
    addCommands(new DriveStraightenCommand(driveSubsystem, 75).withTimeout(0.7));
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_COLLECTING),
        new CollectorCommand(collectorSubsystem).withTimeout(2.5),
        new FeederCommand(feederSubsystem, false),
        new DriveForwardCommand(driveSubsystem).withTimeout(1.2)
      )
    );
    
    addCommands(
      new ParallelCommandGroup(
        new DriveBackwardCommand(driveSubsystem).withTimeout(1.0),
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_SMALL_DISTANCE)
      )
    );
    addCommands(
      new ParallelCommandGroup(
        new DriveStraightenCommand(driveSubsystem, 0).withTimeout(0.7),
        shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem)
      )
    );
    // fourth note
    addCommands(new DriveStraightenCommand(driveSubsystem, -75).withTimeout(0.7));
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_COLLECTING),
        new CollectorCommand(collectorSubsystem).withTimeout(2.5),
        new FeederCommand(feederSubsystem, false),
        new DriveForwardCommand(driveSubsystem).withTimeout(1.2)
      )
    );
    addCommands(
      new ParallelCommandGroup(
        new DriveBackwardCommand(driveSubsystem).withTimeout(1.2),
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_SMALL_DISTANCE)
      )
    );
    addCommands(new DriveStraightenCommand(driveSubsystem, 0).withTimeout(0.7));
    addCommands(new DriveForwardCommand(driveSubsystem).withTimeout(0.1));
    
    
    addCommands(shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem));
  }
  public boolean shouldStraighten() {
    return true;
  }
}
