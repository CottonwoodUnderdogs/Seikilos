// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.OperatorConstants;

/** An example command that uses an example subsystem. */
public class ClimberCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private final GenericHID m_controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberCommand(ClimberSubsystem subsystem, GenericHID controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speedLeft  = m_controller.getRawAxis(OperatorConstants.XboxMappings.LYAxis) * MotorSpeeds.CLIMBER_LEFT_SPEED;
    // negative speedRight so that down on the analog stick is down on the arms.
    double speedRight = -m_controller.getRawAxis(OperatorConstants.XboxMappings.RYAxis) * MotorSpeeds.CLIMBER_RIGHT_SPEED;
    m_subsystem.climbLeft(speedLeft);
    m_subsystem.climbRight(speedRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
