// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AnglerCommand;
import frc.robot.commands.Auto;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CollectorCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ZeroAnglerCommand;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
  private final GenericHID m_driverController = new GenericHID(OperatorConstants.XboxMappings.PRIMARY_CONTROLLER_PORT);
  private final GenericHID m_secondaryController = new GenericHID(OperatorConstants.XboxMappings.SECONDARY_CONTROLLER_PORT);

  // Subsystems
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  public static AnglerSubsystem m_AnglerSubsystem = new AnglerSubsystem();
  private final CollectorSubsystem m_CollectorSubsystem = new CollectorSubsystem();

  // Commands
  private final DriveCommand m_DriveCommand = new DriveCommand(m_DriveSubsystem, m_driverController);

  private final ShooterCommand m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem);
  private final FeederCommand m_FeederCommand = new FeederCommand(m_FeederSubsystem, false);

  private final ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem, m_secondaryController);

  private final AnglerCommand m_AnglerCommand = new AnglerCommand(m_AnglerSubsystem);

  private final CollectorCommand m_CollectorCommand = new CollectorCommand(m_CollectorSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_DriveSubsystem.setDefaultCommand(m_DriveCommand);
    m_ClimberSubsystem.setDefaultCommand(m_ClimberCommand);

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
    // Controller Buttons
    final JoystickButton shooterButton = new JoystickButton(m_secondaryController, OperatorConstants.XboxMappings.A);
    final JoystickButton angleButton = new JoystickButton(m_secondaryController, OperatorConstants.XboxMappings.Y);
    final JoystickButton collectIntakeButton = new JoystickButton(m_driverController, OperatorConstants.XboxMappings.X);

    collectIntakeButton.toggleOnTrue(
      new ParallelCommandGroup(
        new ZeroAnglerCommand(m_AnglerSubsystem),
        new CollectorCommand(m_CollectorSubsystem),
        new FeederCommand(m_FeederSubsystem, false)
      )
    );
    
    angleButton.whileTrue(
      new AnglerCommand(m_AnglerSubsystem)
    );
    
    
    // Start spinning up shooter, wait 3 seconds to get some speed, feed it in.
    shooterButton.onTrue(
      // figuring out how to run multiple motors at the same time
      // took 3 days, we are 5 days from deadline ;-;
      new ParallelCommandGroup(
        new ShooterCommand(m_ShooterSubsystem).withTimeout(2), // the time outs in this sequence are just for turning off the motors after a bit.
        new SequentialCommandGroup(
          new WaitCommand(0.9),
          new FeederCommand(m_FeederSubsystem, true).withTimeout(1)
        )
      )
    );
  
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
