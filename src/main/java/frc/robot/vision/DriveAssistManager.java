package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAssistManager {

    private final SwerveSubsystem swerve;

    /* ================= TARGET DEFINITION ================= */

    // BLUE alliance hub coordinates (replace with actual field measurements)
    private static final Pose2d BLUE_HUB = new Pose2d(
            16.5,   // X
            5.5,    // Y
            new Rotation2d() // facing forward
    );

    private static final double FIELD_LENGTH = 16.54; // total field length in meters

    // Optional: shooter offset from robot center (meters)
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.3, 0.0);

    // Lead time for motion prediction (seconds)
    private static final double LEAD_TIME = 0.25;

    public DriveAssistManager(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    /* ================= ALLIANCE-AWARE TARGET ================= */

    private Pose2d getTarget() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // Flip X and rotate 180° for red alliance
            return new Pose2d(
                    FIELD_LENGTH - BLUE_HUB.getX(),
                    BLUE_HUB.getY(),
                    BLUE_HUB.getRotation().rotateBy(Rotation2d.fromDegrees(180))
            );
        }
        return BLUE_HUB;
    }

    public Pose2d getTargetPose() {
        return getTarget();
    }

    public double getDistanceToTarget() {
        return swerve.getPose()
                .getTranslation()
                .getDistance(getTarget().getTranslation());
    }

    /* ================= AIM CALCULATION ================= */

    public Rotation2d getAimAngle() {
        Pose2d robotPose = swerve.getPose();

        // Current robot velocity
        ChassisSpeeds speeds = swerve.getState().Speeds;

        // Predict future robot position
        double futureX = robotPose.getX() + SHOOTER_OFFSET.getX() + speeds.vxMetersPerSecond * LEAD_TIME;
        double futureY = robotPose.getY() + SHOOTER_OFFSET.getY() + speeds.vyMetersPerSecond * LEAD_TIME;

        Pose2d targetPose = getTarget();

        // Compute angle from predicted robot position to target
        return new Rotation2d(
                Math.atan2(
                        targetPose.getY() - futureY,
                        targetPose.getX() - futureX
                )
        );
    }
}
