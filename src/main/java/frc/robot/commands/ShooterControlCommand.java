package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.DriveAssistManager;

public class ShooterControlCommand extends Command {

    private final ShooterSubsystem shooter;
    private final PreShooterSubsystem preshooter;
    private final DriveAssistManager driveAssist;

    private boolean ready = false;

    public ShooterControlCommand(
        ShooterSubsystem shooter,
        PreShooterSubsystem preshooter,
        DriveAssistManager driveAssist
    ){
        this.shooter = shooter;
        this.preshooter = preshooter;
        this.driveAssist = driveAssist;

        addRequirements(shooter, preshooter);
    }

    @Override
    public void initialize(){
        shooter.arm();
    }

    @Override
    public void execute(){

        double distance =
            driveAssist.getDistanceToTarget();

        shooter.setRPMFromDistance(distance);

        preshooter.followShooter(
            shooter.getTargetRPM()
        );

        ready =
            shooter.readyToFire()
            && preshooter.atSpeed();
    }

    public boolean isReady(){
        return ready;
    }

    @Override
    public void end(boolean interrupted){
        shooter.disarm();
        preshooter.stop();
    }
}