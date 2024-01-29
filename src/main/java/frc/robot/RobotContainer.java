// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;

// Subsystems
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Commands
import frc.robot.commands.elevator.ElevatorMove;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.ActivateLightColour;
//import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems 
  public static final LightSubsystem lightSubsystem = new LightSubsystem();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  //The Robots Commands
  private final ActivateLightColour updateLights = new ActivateLightColour();
  private final ElevatorMove elevatorMove = new ElevatorMove();
  private final IntakeIn intakeIn = new IntakeIn();
  private final IntakeOut intakeOut = new IntakeOut();
  private final ScoreNote scoreNote = new ScoreNote();
  private final SpinUp spinUp = new SpinUp();
  //TODO Amp Command
  //TODO Command to raise and lower the elevator
  //TODO Create command for close speaker shot
  //TODO Trap Command

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final Joystick leftJoystick = new Joystick(OperatorConstants.leftJoystick);
  //private final Joystick rightJoystick = new Joystick(OperatorConstants.rightJoystick);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    lightSubsystem.setDefaultCommand(updateLights);
    elevatorSubsystem.setDefaultCommand(elevatorMove);
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
    operatorController.rightBumper().whileTrue(scoreNote);
    operatorController.leftBumper().whileTrue(spinUp);
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
