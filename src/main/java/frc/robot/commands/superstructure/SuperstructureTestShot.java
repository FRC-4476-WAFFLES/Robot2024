// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

public class SuperstructureTestShot extends Command {
  /** Creates a new SuperstructureTestShot. */
  public SuperstructureTestShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, anglerSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SuperstructureHome home = new SuperstructureHome();
    home.end(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevatorSubsystem.setElevatorTargetPosition(SmartDashboard.getNumber("Elevator Setpoint", 6));
    anglerSubsystem.setAnglerTargetPosition(SmartDashboard.getNumber("Angler Setpoint", 0));
    shooterSubsystem.setShooterTargetSpeed(SmartDashboard.getNumber("Shooter Setpoint", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SuperstructureHome home = new SuperstructureHome();
    // home.schedule();
    shooterSubsystem.setShooterTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
