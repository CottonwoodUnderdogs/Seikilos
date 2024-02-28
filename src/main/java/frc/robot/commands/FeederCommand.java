// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.OperatorConstants;

/** An example command that uses an example subsystem. */
public class FeederCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FeederSubsystem m_subsystem;
  private final boolean m_feedToShooter;
  private boolean timeToFinish = false;
  // private final boolean m_loadIn;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FeederCommand(FeederSubsystem subsystem, boolean feedToShooter) {
    m_subsystem = subsystem;
    m_feedToShooter = feedToShooter;
    // m_loadIn = loadIn;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeToFinish = false;
    SmartDashboard.putBoolean("loaded", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_feedToShooter || !m_subsystem.isNoteLoaded()) {
      m_subsystem.feed(MotorSpeeds.FEEDER_SPEED);
      SmartDashboard.putBoolean("loaded", false);
    } else {
      SmartDashboard.putBoolean("loaded", true);
      timeToFinish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.feed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeToFinish;
  }
}

