// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.superstructure.*;

import static frc.robot.RobotContainer.*;
public class IntakeIn extends Command {
  /** Creates a new IntakeMove. */
  public IntakeIn() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, anglerSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //TODO Add code to only intake until first IR sensor if elevator + angler is at right height and angle

    SuperstructureIntake makeRobotReady = new SuperstructureIntake();

    makeRobotReady.execute();

    if (shooterSubsystem.isNote()) {
        // If a note is in the shooter feeder or intake, stop intake
        intakeSubsystem.SetIntakeSpeed(0);
        shooterSubsystem.setFeederTargetSpeed(0);

    } else if (makeRobotReady.isFinished()) {
      // If robot is ready, intake with feeder
      intakeSubsystem.SetIntakeSpeed(1);
      shooterSubsystem.setFeederTargetSpeed(1);
    }

    //TODO if elevator and angler is at right height, intake using feeder as well.
    //TODO stop spinning feeder if 2nd ir sensor is triggered

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.SetIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
