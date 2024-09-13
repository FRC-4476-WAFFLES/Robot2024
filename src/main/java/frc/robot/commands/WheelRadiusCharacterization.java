package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.ArrayList;
import java.util.List;

public class WheelRadiusCharacterization extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Timer timer = new Timer();
    private final List<Double> angularVelocities = new ArrayList<>();
    private final List<Double> wheelVelocities = new ArrayList<>();
    private double startTime;
    private State state = State.INITIAL;
    private int count = 0;

    private final SwerveRequest.ApplyChassisSpeeds chassisSpeedsRequest = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    private enum State {
        INITIAL, SPEEDING_UP, SLOWING_DOWN, FINISHED
    }

    public WheelRadiusCharacterization(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        startTime = timer.get();
        state = State.INITIAL;
        angularVelocities.clear();
        wheelVelocities.clear();
        count = 0;
    }

    @Override
    public void execute() {
        double time = timer.get();
        double angularVelocity = 0.0;

        switch (state) {
            case INITIAL:
                if (time - startTime > 1.0) {
                    state = State.SPEEDING_UP;
                    startTime = time;
                }
                break;
            case SPEEDING_UP:
                angularVelocity = Math.min((time - startTime) * DriveConstants.maxAngularSpeed / 5.0, DriveConstants.maxAngularSpeed);
                if (angularVelocity >= DriveConstants.maxAngularSpeed) {
                    state = State.SLOWING_DOWN;
                    startTime = time;
                }
                break;
            case SLOWING_DOWN:
                angularVelocity = Math.max(DriveConstants.maxAngularSpeed - (time - startTime) * DriveConstants.maxAngularSpeed / 5.0, 0.0);
                if (angularVelocity <= 0.0) {
                    state = State.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }

        driveSubsystem.setControl(chassisSpeedsRequest.withSpeeds(new ChassisSpeeds(0, 0, angularVelocity)));

        if (count % 5 == 0) {
            angularVelocities.add(angularVelocity);
            wheelVelocities.add(driveSubsystem.getAverageWheelVelocity());
        }
        count++;

        SmartDashboard.putString("Characterization State", state.toString());
        SmartDashboard.putNumber("Characterization Angular Velocity", angularVelocity);
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setControl(chassisSpeedsRequest.withSpeeds(new ChassisSpeeds()));
        timer.stop();

        if (!interrupted) {
            double wheelRadius = calculateWheelRadius();
            SmartDashboard.putNumber("Estimated Wheel Radius", wheelRadius);
            System.out.println("Estimated Wheel Radius: " + wheelRadius);
        }
    }

    private double calculateWheelRadius() {
        double sumRadius = 0.0;
        int numSamples = 0;

        for (int i = 0; i < angularVelocities.size(); i++) {
            double angularVelocity = angularVelocities.get(i);
            double wheelVelocity = wheelVelocities.get(i);

            if (Math.abs(angularVelocity) > 0.1) {
                sumRadius += wheelVelocity / angularVelocity;
                numSamples++;
            }
        }

        return sumRadius / numSamples;
    }
}
