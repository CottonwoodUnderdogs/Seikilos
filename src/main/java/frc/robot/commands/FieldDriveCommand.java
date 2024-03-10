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
import frc.robot.Constants.OperatorConstants.XboxMappings;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class FieldDriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final GenericHID m_controller;

  /**
   * Creates a new AnglerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FieldDriveCommand(DriveSubsystem subsystem, GenericHID controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setDefaultCommand(this);
    RobotContainer.m_DriveSubsystem.zeroGyro();
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vx = -m_controller.getRawAxis(XboxMappings.LYAxis);
    double vy = m_controller.getRawAxis(XboxMappings.LXAxis);
    double omega = m_controller.getRawAxis(XboxMappings.RXAxis);

    m_subsystem.driveFieldCartesian(vx, vy, omega);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
