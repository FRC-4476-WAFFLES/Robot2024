// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;

// Subsystems
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.AnglerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Telemetry;
// Commands
import frc.robot.commands.elevator.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.superstructure.*;
import frc.robot.commands.ActivateLightColour;
import frc.robot.commands.drive.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick leftJoystick = new CommandJoystick(OperatorConstants.leftJoystick);
  private final CommandJoystick rightJoystick = new CommandJoystick(OperatorConstants.rightJoystick);
  public static final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public double rightTriggerStrength = 1;
  public double leftTriggerStrength = 1;

  // The robot's subsystems 
  public static final LightSubsystem lightSubsystem = null; //new LightSubsystem();
  public static final ElevatorSubsystem elevatorSubsystem = null; // new ElevatorSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ShooterSubsystem shooterSubsystem  = new ShooterSubsystem();
  public static final AnglerSubsystem anglerSubsystem  = new AnglerSubsystem();
  public static final DriveSubsystem driveSubsystem = TunerConstants.DriveTrain;

  //The Robots Commands
  private final ActivateLightColour updateLights = null; // = new ActivateLightColour();
  private final ElevatorUp elevatorUp  = null; //= new ElevatorUp();
  private final ElevatorDown elevatorDown  = null; //= new ElevatorDown();
  private final IntakeIn intakeIn = new IntakeIn();
  private final IntakeOut intakeOut = new IntakeOut();
  private final ScoreNote scoreNote  = null; //= new ScoreNote();
  private final SpinUp spinUp  = null; //= new SpinUp();
  private final SuperstructureHome superstructureHome  = null; //= new SuperstructureHome();
  private final SuperstructureAmp superstructureAmp = null; // = new SuperstructureAmp();
  private final SuperstructureCloseSpeaker superstructureCloseSpeaker  = null; //= new SuperstructureCloseSpeaker();
  private final SuperstructureIntake superstructureIntake  = null; //= new SuperstructureIntake();
  

  //TODO Trap Command

  private final DriveTeleop driveTeleop = new DriveTeleop(
    () -> -leftJoystick.getY() * DriveConstants.maxSpeed, 
    () -> -leftJoystick.getX() * DriveConstants.maxSpeed, 
    () -> -rightJoystick.getX() * DriveConstants.maxAngularSpeed
  );

  // private final DriveAndPointAtTarget driveAndAimAtGoal = new DriveAndPointAtTarget(() -> leftJoystick.getY() * DriveConstants.maxSpeed, () -> leftJoystick.getX() * DriveConstants.maxSpeed, driveSubsystem::getAngleToGoal);

  private final SendableChooser<Command> autoChooser;

  private final Telemetry logger = new Telemetry(DriveConstants.maxSpeed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register named commands for auto
    registerNamedCommands();
    // Configure the trigger bindings
    configureBindings();

    // lightSubsystem.setDefaultCommand(updateLights);
    driveSubsystem.setDefaultCommand(driveTeleop);

    driveSubsystem.registerTelemetry(logger::telemeterize);

    // Build an auto chooser. This will use Commands.none() as the default option.
    // Another option is to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    

    operatorController.povLeft().whileTrue(intakeIn);
    operatorController.povRight().whileTrue(intakeOut);
    // operatorController.rightBumper().whileTrue(scoreNote);
    // operatorController.leftBumper().whileTrue(spinUp);
    // operatorController.x().whileTrue(superstructureAmp);
    // operatorController.b().whileTrue(superstructureCloseSpeaker);
    // rightJoystick.button(1).whileTrue(driveAndAimAtGoal);
    // operatorController.rightTrigger().whileTrue(elevatorUp);
    // operatorController.leftTrigger().whileTrue(elevatorDown);

  }

  /**
   * Use this method to define name->command mappings. Names will be used by PathPlanner to 
   * call commands in full autos. 
   */
  private void registerNamedCommands() {
    // Register Named Commands
    // Add other commands to be able to run them in autos
    // NamedCommands.registerCommand("intakeIn", intakeIn);
    // NamedCommands.registerCommand("scoreNote", scoreNote);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
