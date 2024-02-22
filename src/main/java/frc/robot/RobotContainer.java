// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AnglerCommand;
import frc.robot.commands.AnglerOffCommand;
import frc.robot.commands.Auto;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CollectorCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final GenericHID m_driverController = new GenericHID(OperatorConstants.SwitchMappings.kDriverControllerPort);

  // Subsystems
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final AnglerSubsystem m_AnglerSubsystem = new AnglerSubsystem();
  private final CollectorSubsystem m_CollectorSubsystem = new CollectorSubsystem();

  // Commands
  private final DriveCommand m_DriveCommand = new DriveCommand(m_DriveSubsystem, m_driverController);

  private final ShooterCommand m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem);
  private final FeederCommand m_FeederCommand = new FeederCommand(m_ShooterSubsystem);

  private final ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem, m_driverController);

  private final AnglerCommand m_AnglerCommand = new AnglerCommand(m_AnglerSubsystem);

  private final CollectorCommand m_CollectorCommand = new CollectorCommand(m_CollectorSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_DriveSubsystem.setDefaultCommand(m_DriveCommand);
    m_ClimberSubsystem.setDefaultCommand(m_ClimberCommand);
    // m_AnglerSubsystem.setDefaultCommand(m_AnglerCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // testing motors
    final JoystickButton shooterButton = new JoystickButton(m_driverController, OperatorConstants.SwitchMappings.A);
    final JoystickButton feederButton = new JoystickButton(m_driverController, OperatorConstants.SwitchMappings.Y);

    final JoystickButton angleButton = new JoystickButton(m_driverController, OperatorConstants.SwitchMappings.B);

    final JoystickButton collectIntakeButton = new JoystickButton(m_driverController, OperatorConstants.SwitchMappings.X);
    
    shooterButton.whileTrue(m_ShooterCommand);
    collectIntakeButton.whileTrue(m_CollectorCommand);
    feederButton.whileTrue(m_FeederCommand);
    angleButton.onTrue(m_AnglerCommand).onFalse(new AnglerOffCommand(m_AnglerSubsystem));
    

    // angleButton.whileTrue(m_AnglerCommand);
    // angleButton.onTrue(
    //   new SequentialCommandGroup(
    //     Commands.print("angling"),
    //     m_AnglerCommand,
    //     Commands.print("waiting"),
    //     new WaitCommand(3),
    //     Commands.print("end angling"),
    //     new AnglerOffCommand(m_AnglerSubsystem)
    //   )
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Auto.exampleAuto(null);
  }
}
