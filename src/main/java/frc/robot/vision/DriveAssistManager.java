package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAssistManager {

    private final SwerveSubsystem swerve;

    // ======== Field / Hub Settings ========
    private static final Pose2d BLUE_HUB =
        new Pose2d(4.623, 4.033, new Rotation2d(0));
//X+ -> RED SIDE  Y+ -> LEFT FROM BLUE PERS.
    private static final double FIELD_LENGTH = 16.54;

    // ======== Shooter Offset & Lead ========
    // Positive X points forward; if shooter is behind, use negative X
    private static final Translation2d SHOOTER_OFFSET =
        new Translation2d(-0.3, 0.0); // 30cm behind robot center

    private static final double LEAD_TIME = 0.5; // seconds for predictive aiming

    // ======== Aim Settings ========
    private static final double AIM_TOLERANCE_DEG = 0.25; // degrees

    public DriveAssistManager(SwerveSubsystem s) {
        this.swerve = s;
    }

    // ======== Alliance-adjusted hub target ========
    private Pose2d getTarget() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() &&
            alliance.get() == DriverStation.Alliance.Red) {

            return new Pose2d(
                FIELD_LENGTH - BLUE_HUB.getX(),
                BLUE_HUB.getY(),
                BLUE_HUB.getRotation().rotateBy(Rotation2d.k180deg)
            );
        }

        return BLUE_HUB;
    }

    // ======== Compute Aim Angle for Shooter ========
    public Rotation2d getAimAngle() {

        Pose2d robotPose = swerve.getPose();

        // Convert robot-relative speeds to field-relative
        ChassisSpeeds fieldSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                swerve.getState().Speeds,
                robotPose.getRotation()
            );

        // Shooter position on the field
        Translation2d shooterPos =
            robotPose.getTranslation().plus(
                SHOOTER_OFFSET.rotateBy(robotPose.getRotation())
            );

        // Predict future position (if using lead compensation)
        Translation2d futurePos =
            shooterPos.plus(
                new Translation2d(
                    fieldSpeeds.vxMetersPerSecond * LEAD_TIME,
                    fieldSpeeds.vyMetersPerSecond * LEAD_TIME
                )
            );

        // Angle from shooter to hub
        Rotation2d angleToTarget =
            getTarget().getTranslation().minus(futurePos).getAngle();

        // Rotate 180° because shooter is mounted on back
        return angleToTarget.rotateBy(Rotation2d.k180deg);
    }

    // ======== Returns true if robot is aimed within tolerance ========
    public boolean isAimed() {
        double errorDeg = getAimAngle()
            .minus(swerve.getPose().getRotation())
            .getDegrees();

        // Normalize -180 -> 180
        errorDeg = Math.atan2(Math.sin(Math.toRadians(errorDeg)),
                              Math.cos(Math.toRadians(errorDeg))) 
                   * 180.0 / Math.PI;

        return Math.abs(errorDeg) <= AIM_TOLERANCE_DEG;
    }

    // ======== Distance from robot center to target ========
    public double getDistanceToTarget() {
        return swerve.getPose()
            .getTranslation()
            .getDistance(getTarget().getTranslation());
    }

    // ======== Return target pose for visualization ========
    public Pose2d getTargetPose() {
        return getTarget();
    }
}