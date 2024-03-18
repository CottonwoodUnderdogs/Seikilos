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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        new DriveStraightenCommand(driveSubsystem, 0).withTimeout(0.6)
      )
    );
    // second note
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_COLLECTING),
        new FeederCommand(feederSubsystem, false).deadlineWith(new CollectorCommand(collectorSubsystem)),
        new DriveForwardCommand(driveSubsystem).withTimeout(0.6)
      )
    );
    addCommands(
      new ParallelCommandGroup(
        new DriveStraightenCommand(driveSubsystem,0).withTimeout(0.4),
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_DIRECT)
      )
    );
    addCommands(
      new ParallelCommandGroup(
        new DriveBackwardCommand(driveSubsystem).withTimeout(0.6),
        shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem),
        new WaitCommand(1.5)
      )
    );
    // third note
    addCommands(new DriveStraightenCommand(driveSubsystem, 42.5).withTimeout(1));
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_DIRECT),
        new FeederCommand(feederSubsystem, false).deadlineWith(new CollectorCommand(collectorSubsystem)),
        new DriveForwardCommand(driveSubsystem).withTimeout(0.83)
      )
    );
    
    addCommands(
        new DriveBackwardCommand(driveSubsystem).withTimeout(0.55)
        
    );
    addCommands(new DriveStraightenCommand(driveSubsystem, 0).withTimeout(1));
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_DIRECT),
        new DriveBackwardCommand(driveSubsystem).withTimeout(0.35),
        shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem)
      )

    );
    // fourth note
    addCommands(new DriveStraightenCommand(driveSubsystem, -50).withTimeout(1.5));
    addCommands(
      new ParallelCommandGroup(
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_COLLECTING),
        new FeederCommand(feederSubsystem, false).deadlineWith(new CollectorCommand(collectorSubsystem)),
        new DriveForwardCommand(driveSubsystem).withTimeout(0.7)
      )
    );
    addCommands(
      new ParallelCommandGroup(
        new DriveBackwardCommand(driveSubsystem).withTimeout(0.5),
        new SetAnglerCommand(anglerSubsystem, SetPoints.ANGLER_SMALL_DISTANCE)
      )
    );
    
    addCommands(new DriveStraightenCommand(driveSubsystem, 0).withTimeout(1));
    addCommands(
      new ParallelCommandGroup(
        new DriveBackwardCommand(driveSubsystem).withTimeout(0.2),
        shooterSubsystem.shootSequence(feederSubsystem, shooterSubsystem)
      )
    );
  }
  public boolean shouldStraighten() {
    return true;
  }
}
