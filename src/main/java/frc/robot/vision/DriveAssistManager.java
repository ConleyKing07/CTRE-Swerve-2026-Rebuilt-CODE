package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAssistManager {

    private final SwerveSubsystem swerve;

    // ================= TARGET SELECTION =================
    public enum AimTarget {
        HUB,
        PASS_LEFT,
        PASS_RIGHT
    }

    private AimTarget currentTarget = AimTarget.HUB;

    public void setTarget(AimTarget target) {
        currentTarget = target;
    }

    // ================= FIELD SETTINGS =================
    // X+ -> RED SIDE   Y+ -> LEFT FROM BLUE PERSPECTIVE

    // Main scoring target
    private static final Pose2d BLUE_HUB =
        new Pose2d(4.623, 4.033, new Rotation2d(0));

    // Passing locations (example coordinates — replace with real ones)
    private static final Pose2d BLUE_PASS_LEFT =
        new Pose2d(2.00, 6.5, new Rotation2d(0));

    private static final Pose2d BLUE_PASS_RIGHT =
        new Pose2d(2.00, 1.5, new Rotation2d(0));

    private static final double FIELD_LENGTH = 16.54;

    // ================= SHOOTER OFFSET =================
    // If shooter is mounted behind robot center, X should be negative
    private static final Translation2d SHOOTER_OFFSET =
        new Translation2d(-0.3, 0.0);

    // ================= LEAD SETTINGS =================
    private static final boolean USE_LEAD = true;
    private static final double LEAD_TIME = 0.85;//seconds

    // ================= AIM SETTINGS =================
    private static final double AIM_TOLERANCE_DEG = 0.05;

    public DriveAssistManager(SwerveSubsystem s) {
        this.swerve = s;
    }

    // ==================================================
    // Alliance-adjusted target (based on selected mode)
    // ==================================================
    private Pose2d getTarget() {

        Pose2d blueTarget;

        switch (currentTarget) {
            case PASS_LEFT:
                blueTarget = BLUE_PASS_LEFT;
                break;

            case PASS_RIGHT:
                blueTarget = BLUE_PASS_RIGHT;
                break;

            case HUB:
            default:
                blueTarget = BLUE_HUB;
                break;
        }

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() &&
            alliance.get() == DriverStation.Alliance.Red) {

            return new Pose2d(
                FIELD_LENGTH - blueTarget.getX(),
                blueTarget.getY(),
                blueTarget.getRotation().rotateBy(Rotation2d.k180deg)
            );
        }

        return blueTarget;
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

        // Apply motion lead if enabled
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

    public boolean isAimedDR() {
        return Math.abs(getAimErrorDeg()) <= 3.0;
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
        SmartDashboard.putBoolean("Aimed", isAimedDR());
        SmartDashboard.putString("Aim Target", currentTarget.name());
    }

    // ==================================================
    // Target Pose for Field Visualization
    // ==================================================
    public Pose2d getTargetPose() {
        return getTarget();
    }
}
