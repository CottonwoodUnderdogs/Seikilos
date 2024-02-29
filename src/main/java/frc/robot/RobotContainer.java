// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AnglerCommand;
import frc.robot.commands.AnglerDownCommand;
import frc.robot.commands.AnglerPresetDirectCommand;
import frc.robot.commands.AnglerPresetSideCommand;
import frc.robot.commands.AutoDirect;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CollectorCommand;
import frc.robot.commands.CollectorOutCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SlowCommand;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.commands.ZeroAnglerCommand;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.lang.reflect.Field;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  public static DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  public static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public static FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  public static AnglerSubsystem m_AnglerSubsystem = new AnglerSubsystem();
  public static CollectorSubsystem m_CollectorSubsystem = new CollectorSubsystem();

  // Commands
  private final DriveCommand m_DriveCommand = new DriveCommand(m_DriveSubsystem, m_driverController);
  private final FieldDriveCommand m_FieldDriveCommand = new FieldDriveCommand(m_DriveSubsystem, m_driverController);

  private final ShooterCommand m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem);
  private final FeederCommand m_FeederCommand = new FeederCommand(m_FeederSubsystem, false);

  private final ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem, m_secondaryController);

  private final AnglerCommand m_AnglerCommand = new AnglerCommand(m_AnglerSubsystem);

  private final CollectorCommand m_CollectorCommand = new CollectorCommand(m_CollectorSubsystem);

  private final Command m_AutoDirect = new AutoDirect(m_DriveSubsystem, m_FeederSubsystem, m_ShooterSubsystem, m_AnglerSubsystem, m_CollectorSubsystem);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
    


  public static boolean boolfieldOriented = true;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_chooser.setDefaultOption("direct shoot", m_AutoDirect);
    SmartDashboard.putData(m_chooser);

    m_DriveSubsystem.setDefaultCommand(m_FieldDriveCommand);
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
    // Controller 1
    final JoystickButton collectIntakeButton = new JoystickButton(m_driverController, OperatorConstants.XboxMappings.R);
    final JoystickButton collectOuttakeButton = new JoystickButton(m_driverController, OperatorConstants.XboxMappings.L);
    final JoystickButton slowModeButton = new JoystickButton(m_driverController, OperatorConstants.XboxMappings.A);
    final JoystickButton fieldOrientedButton = new JoystickButton(m_driverController, OperatorConstants.XboxMappings.B);
    final JoystickButton roboOrientedButton = new JoystickButton(m_driverController, OperatorConstants.XboxMappings.Y);

    
    collectIntakeButton.toggleOnTrue(
      new ParallelRaceGroup(
        new ZeroAnglerCommand(m_AnglerSubsystem),
        new CollectorCommand(m_CollectorSubsystem),
        new FeederCommand(m_FeederSubsystem, false)
      )
    );
    collectOuttakeButton.whileTrue(new CollectorOutCommand(m_CollectorSubsystem));
    fieldOrientedButton.onTrue(new FieldDriveCommand(m_DriveSubsystem, m_driverController));
    roboOrientedButton.onTrue(new DriveCommand(m_DriveSubsystem, m_driverController));
    slowModeButton.onTrue(new SlowCommand(m_DriveSubsystem));
    
    // Controller 2
    final JoystickButton shooterButton = new JoystickButton(m_secondaryController, OperatorConstants.XboxMappings.X);
    final JoystickButton angleButton = new JoystickButton(m_secondaryController, OperatorConstants.XboxMappings.R);
    final JoystickButton angleDownButton = new JoystickButton(m_secondaryController, OperatorConstants.XboxMappings.L);
    final JoystickButton anglePresetDirect = new JoystickButton(m_secondaryController, OperatorConstants.XboxMappings.B);
    final JoystickButton anglePresetSide = new JoystickButton(m_secondaryController, OperatorConstants.XboxMappings.A);
    
    // Start spinning up shooter, wait 3 seconds to get some speed, feed it in.
    shooterButton.onTrue(
      // figuring out how to run multiple motors at the same time
      // took 3 days, we are 5 days from deadline ;-;
      new ParallelCommandGroup(
        new ShooterCommand(m_ShooterSubsystem).withTimeout(5), // the time outs in this sequence are just for turning off the motors after a bit.
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new FeederCommand(m_FeederSubsystem, true).withTimeout(1)
        )
      )
    );
    angleButton.whileTrue(new AnglerCommand(m_AnglerSubsystem));
    angleDownButton.whileTrue(new AnglerDownCommand(m_AnglerSubsystem));
    anglePresetDirect.onTrue(new AnglerPresetDirectCommand(m_AnglerSubsystem));
    anglePresetSide.onTrue(new AnglerPresetSideCommand(m_AnglerSubsystem));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return AutoDirect.exampleAuto(null);
    return m_chooser.getSelected();
  }
}
