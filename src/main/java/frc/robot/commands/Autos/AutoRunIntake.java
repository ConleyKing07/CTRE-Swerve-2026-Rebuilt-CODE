package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoRunIntake extends Command {

    private final IntakeSubsystem intake;
    private final double speed;
    private final double duration;
    private final Timer timer = new Timer();

    public AutoRunIntake(
            IntakeSubsystem intake,
            double speed,
            double durationSeconds) {

        this.intake = intake;
        this.speed = speed;
        this.duration = durationSeconds;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intake.intakeRun(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
