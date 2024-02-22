// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.superstructure.SuperstructureHome;
import frc.robot.commands.superstructure.SuperstructureIntake;

import static frc.robot.RobotContainer.*;

public class IntakeOut extends Command {
  /** Creates a new IntakeOut. */
  public IntakeOut() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.SetIntakeSpeed(-1.0);
    shooterSubsystem.setFeederTargetSpeed(-100);
    SuperstructureIntake superstructureIntakePosition = new SuperstructureIntake();
    superstructureIntakePosition.execute();
    // TODO if elevator and angler is in intake position outtake the feeder as well.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     intakeSubsystem.SetIntakeSpeed(0);
     shooterSubsystem.setFeederTargetSpeed(0);
     SuperstructureHome superstructureHomePosition = new SuperstructureHome();
     superstructureHomePosition.execute();
     //shooterSubsystem.setFeederTargetSpeed(-1);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
