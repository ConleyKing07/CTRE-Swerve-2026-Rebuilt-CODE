package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class AutoFireCommand extends Command {

    private final HopperSubsystem hopper;
    private final double duration;
    private final Timer timer = new Timer();

    public AutoFireCommand(
            HopperSubsystem hopper,
            double durationSeconds) {

        this.hopper = hopper;
        this.duration = durationSeconds;

        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        hopper.feed();
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
