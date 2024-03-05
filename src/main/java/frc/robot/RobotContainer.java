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
import frc.robot.subsystems.FeederSubsystem;
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  public static final LightSubsystem lightSubsystem = new LightSubsystem();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ShooterSubsystem shooterSubsystem  = new ShooterSubsystem();
  public static final AnglerSubsystem anglerSubsystem  = new AnglerSubsystem();
  public static final FeederSubsystem feederSubsystem  = new FeederSubsystem();
  public static final DriveSubsystem driveSubsystem = TunerConstants.DriveTrain;

  //The Robots Commands
  private final DriveAndPointAtTarget driveAndAimAtGoal = new DriveAndPointAtTarget(() -> leftJoystick.getY() * DriveConstants.maxSpeed, () -> leftJoystick.getX() * DriveConstants.maxSpeed, driveSubsystem::getAngleToGoal);
  private final DriveAndPointAtTarget driveAndAimAtGoalAuto = new DriveAndPointAtTarget(() -> leftJoystick.getY() * DriveConstants.maxSpeed, () -> leftJoystick.getX() * DriveConstants.maxSpeed, driveSubsystem::getAngleToGoal);
  private final ActivateLightColour updateLights = new ActivateLightColour();
  private final ElevatorUp elevatorUp  = null; //= new ElevatorUp();
  private final ElevatorDown elevatorDown  = null; //= new ElevatorDown();
  private final IntakeIn intakeIn = new IntakeIn();
  private final IntakeIn intakeInAuto = new IntakeIn();
  private final IntakeOut intakeOut = new IntakeOut();
  private final ScoreNote scoreNote  = new ScoreNote();
  private final SpinUp spinUp  = new SpinUp();
  private final ScoreNote scoreNoteAuto  = new ScoreNote();
  private final SpinUp spinUpAuto  = new SpinUp();
  private final SuperstructureHome superstructureHome = new SuperstructureHome();
  private final SuperstructureAmp superstructureAmp = new SuperstructureAmp();
  private final SuperstructureCloseSpeaker superstructureCloseSpeaker  = new SuperstructureCloseSpeaker();
  private final SuperstructureIntake superstructureIntake  = new SuperstructureIntake();
  private final SuperstructureTestShot superstructureTestShot = new SuperstructureTestShot();
  private final BasicFieldReset basicFieldReset = new BasicFieldReset();
  private final ResetGyro resetGyro = new ResetGyro();
  private final SuperstructureClimb superstructureClimb = new SuperstructureClimb();
  private final SuperstructureStash superstructureStash = new SuperstructureStash();

  


  //TODO Trap Command
  
 

  
  private final SendableChooser<Command> autoChooser;
  private static RobotContainer containerRobot;

  private final Telemetry logger = new Telemetry(DriveConstants.maxSpeed);
  private DriveTeleop driveTeleopRed;
  private DriveTeleop driveTeleopBlue;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){
    containerRobot = this;
    driveTeleopBlue = new DriveTeleop(
    () -> -leftJoystick.getY() * DriveConstants.maxSpeed, 
    () -> -leftJoystick.getX() * DriveConstants.maxSpeed, 
    () -> -rightJoystick.getX() * DriveConstants.maxAngularSpeed
    );
    driveTeleopRed = new DriveTeleop(
    () -> leftJoystick.getY() * DriveConstants.maxSpeed, 
    () -> leftJoystick.getX() * DriveConstants.maxSpeed, 
    () -> -rightJoystick.getX() * DriveConstants.maxAngularSpeed
  );
  
    // Register named commands for auto
    registerNamedCommands();
    // Configure the trigger bindings
    configureBindings();

    //driveSubsystem.setDefaultCommand(driveTeleop);
    setAllianceColor();
    lightSubsystem.setDefaultCommand(updateLights);
    elevatorSubsystem.setDefaultCommand(superstructureHome);


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
    operatorController.rightBumper().whileTrue(scoreNote);
    operatorController.x().whileTrue(superstructureAmp);
    operatorController.b().whileTrue(superstructureCloseSpeaker);
    operatorController.y().whileTrue(superstructureTestShot);
    operatorController.a().whileTrue(superstructureClimb);
    operatorController.rightStick().whileTrue(superstructureStash);
    rightJoystick.button(1).whileTrue(driveAndAimAtGoal.alongWith(spinUp));
    // operatorController.rightTrigger().whileTrue(elevatorUp);
    // operatorController.leftTrigger().whileTrue(elevatorDown);

    rightJoystick.button(8).whileTrue(basicFieldReset);
    leftJoystick.button(8).whileTrue(basicFieldReset);

    rightJoystick.button(9).whileTrue(resetGyro);
    leftJoystick.button(9).whileTrue(resetGyro);
  }

  /**
   * Use this method to define name->command mappings. Names will be used by PathPlanner to 
   * call commands in full autos. 
   */
  private void registerNamedCommands() {
    // Register Named Commands
    // Add other commands to be able to run them in autos
    // NamedCommands.registerCommand("aimAtGoal", driveAndAimAtGoal);
    // NamedCommands.registerCommand("spinUp", spinUp);
    // NamedCommands.registerCommand("scoreNote", scoreNote);
    NamedCommands.registerCommand("intakeIn", intakeInAuto);
    NamedCommands.registerCommand("intakeInDeadline", intakeInAuto.withTimeout(1));
    NamedCommands.registerCommand("intakeOff", new InstantCommand(() -> intakeIn.cancel()));
    NamedCommands.registerCommand("completeShot", Commands.deadline(scoreNoteAuto, spinUpAuto, driveAndAimAtGoalAuto));
  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static void setAllianceColor(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue){
        driveSubsystem.setDefaultCommand(containerRobot.driveTeleopBlue);
      }
      else if (alliance.get() == Alliance.Red){
        driveSubsystem.setDefaultCommand(containerRobot.driveTeleopRed);
      }
    }
  }
}
