package frc.robot.vision;

import frc.robot.subsystems.*;

public class AutoScoringManager {
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    public AutoScoringManager(ShooterSubsystem shooter,
                              HopperSubsystem hopper,
                              DriveAssistManager driveAssist) {
        this.shooter = shooter;
        this.hopper = hopper;
    }

    public boolean readyToShoot() {
        return shooter.isArmed() && shooter.atSpeed();
    }

    public void update() {
        if (readyToShoot()) {
            hopper.feed();
        } else {
            hopper.stop();
        }
    }
}
