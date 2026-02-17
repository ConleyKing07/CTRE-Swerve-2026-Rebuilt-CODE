package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Outtake extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

     public Outtake(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeOut(speed);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeOut(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}