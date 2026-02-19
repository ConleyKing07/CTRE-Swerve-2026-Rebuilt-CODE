package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAssistManager {

    private final SwerveSubsystem swerve;

    // ================= FIELD SETTINGS =================
    // X+ -> RED SIDE   Y+ -> LEFT FROM BLUE PERSPECTIVE
    private static final Pose2d BLUE_HUB =
        new Pose2d(4.623, 4.033, new Rotation2d(0));

    private static final double FIELD_LENGTH = 16.54;

    // ================= SHOOTER OFFSET =================
    // If shooter is mounted behind robot center, X should be negative
    private static final Translation2d SHOOTER_OFFSET =
        new Translation2d(-0.3, 0.0);

    // ================= LEAD SETTINGS =================
    private static final boolean USE_LEAD = true;
    private static final double LEAD_TIME = 0.5; // seconds

    // ================= AIM SETTINGS =================
    private static final double AIM_TOLERANCE_DEG = 0.25;

    public DriveAssistManager(SwerveSubsystem s) {
        this.swerve = s;
    }

    // ==================================================
    // Alliance-adjusted target
    // ==================================================
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

    // ==================================================
    // Compute Aim Angle for Shooter
    // ==================================================
    public Rotation2d getAimAngle() {

        Pose2d robotPose = swerve.getPose();

        // Convert robot-relative speeds to field-relative
        ChassisSpeeds fieldSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                swerve.getState().Speeds,
                robotPose.getRotation()
            );

        // Shooter position on field
        Translation2d shooterPos =
            robotPose.getTranslation().plus(
                SHOOTER_OFFSET.rotateBy(robotPose.getRotation())
            );

        Translation2d predictedPos = shooterPos;

        if (USE_LEAD) {
            predictedPos = shooterPos.plus(
                new Translation2d(
                    fieldSpeeds.vxMetersPerSecond * LEAD_TIME,
                    fieldSpeeds.vyMetersPerSecond * LEAD_TIME
                )
            );
        }

        Rotation2d angleToTarget =
            getTarget().getTranslation()
                .minus(predictedPos)
                .getAngle();

        // Rotate 180° because shooter faces backwards
        return angleToTarget.rotateBy(Rotation2d.k180deg);
    }

    // ==================================================
    // Aim Error (degrees)
    // ==================================================
    public double getAimErrorDeg() {

        double errorDeg = getAimAngle()
            .minus(swerve.getPose().getRotation())
            .getDegrees();

        // Normalize to -180 to 180
        errorDeg = Math.atan2(
                Math.sin(Math.toRadians(errorDeg)),
                Math.cos(Math.toRadians(errorDeg))
            ) * 180.0 / Math.PI;

        return errorDeg;
    }

    // ==================================================
    // Aimed Check
    // ==================================================
    public boolean isAimed() {
        return Math.abs(getAimErrorDeg()) <= AIM_TOLERANCE_DEG;
    }

    // ==================================================
    // Distance to Target
    // ==================================================
    public double getDistanceToTarget() {
        return swerve.getPose()
            .getTranslation()
            .getDistance(getTarget().getTranslation());
    }

    // ==================================================
    // Publish Everything to DriverStation
    // ==================================================
    public void publishTelemetry() {

        double meters = getDistanceToTarget();
        double feet = meters * 3.28084;
        double error = getAimErrorDeg();

        SmartDashboard.putNumber("Target Distance (m)", meters);
        SmartDashboard.putNumber("Target Distance (ft)", feet);
        SmartDashboard.putNumber("Aim Error (deg)", error);
        SmartDashboard.putBoolean("Aimed", isAimed());
    }

    // ==================================================
    // Target Pose for Field Visualization
    // ==================================================
    public Pose2d getTargetPose() {
        return getTarget();
    }
}
