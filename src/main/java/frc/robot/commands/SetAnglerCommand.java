// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.CollectorSubsystem;

/** An example command that uses an example subsystem. */
public class SetAnglerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AnglerSubsystem m_subsystem;
  private final double m_rotations;
  private boolean timeToFinish = false;

  /**
   * Creates a new AnglerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetAnglerCommand(AnglerSubsystem subsystem, double rotations) {
    m_subsystem = subsystem;
    m_rotations = rotations;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setAngle(m_rotations);
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timeToFinish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeToFinish;
  }
}
