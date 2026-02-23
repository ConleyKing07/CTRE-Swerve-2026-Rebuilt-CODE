package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.DriveAssistManager;

public class AutoSpinUp extends Command {

    private final ShooterSubsystem shooter;
    private final PreShooterSubsystem preshooter;
    private final DriveAssistManager driveAssist;

    private final Timer timer = new Timer();
    private final double durationSeconds;

    public AutoSpinUp(
        ShooterSubsystem shooter,
        PreShooterSubsystem preshooter,
        DriveAssistManager driveAssist,
        double durationSeconds) {

        this.shooter = shooter;
        this.preshooter = preshooter;
        this.driveAssist = driveAssist;
        this.durationSeconds = durationSeconds;

        addRequirements(shooter, preshooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shooter.arm();
    }

    @Override
    public void execute() {
        double distance = driveAssist.getDistanceToTarget();

        shooter.setRPMFromDistance(distance);

        preshooter.followShooter(
            shooter.getTargetRPM());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disarm();
        preshooter.stop();
        timer.stop();
    }
}
