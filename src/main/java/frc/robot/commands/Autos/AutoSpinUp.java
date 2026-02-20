package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.DriveAssistManager;

public class AutoSpinUp extends Command {

    private final ShooterSubsystem shooter;
    private final PreShooterSubsystem preshooter;
    private final DriveAssistManager driveAssist;

    public AutoSpinUp(
        ShooterSubsystem shooter,
        PreShooterSubsystem preshooter,
        DriveAssistManager driveAssist) {

        this.shooter = shooter;
        this.preshooter = preshooter;
        this.driveAssist = driveAssist;

        addRequirements(shooter, preshooter);
    }

    @Override
    public void initialize() {
        shooter.arm();
    }

    @Override
    public void execute() {

        double distance =
            driveAssist.getDistanceToTarget();

        shooter.setRPMFromDistance(distance);

        preshooter.followShooter(
            shooter.getTargetRPM());
    }

    @Override
    public boolean isFinished() {
        return shooter.readyToFire()
            && preshooter.atSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.disarm();
            preshooter.stop();
        }
    }
}
