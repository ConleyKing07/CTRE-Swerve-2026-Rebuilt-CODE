package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;
import frc.robot.Telemetry;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {

    // ================= HEADING HOLD =================
    public Rotation2d targetHeading = Rotation2d.kZero;
    public boolean headingHoldEnabled = false;
    public static final double kP_heading = 0.08;

    // ================= LIMELIGHT SETTINGS =================
    private static final String LIMELIGHT_NAME = "limelight-shooter";
    private static final double MAX_VISION_AMBIGUITY = 0.7;

    // ================= SIMULATION =================
    private Notifier simNotifier;
    private double lastSimTime;
    private final Field2d field = new Field2d();

    private void startSim() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            double now = Utils.getCurrentTimeSeconds();
            double dt = now - lastSimTime;
            lastSimTime = now;
            updateSimState(dt, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(0.005);
    }

    private final Telemetry telemetry =
        new Telemetry(
            TunerConstants.kSpeedAt12Volts
                .in(edu.wpi.first.units.Units.MetersPerSecond)
        );

    // ================= OPERATOR PERSPECTIVE =================
    private boolean perspectiveApplied = false;
    private static final Rotation2d kBlueForward = Rotation2d.kZero;
    private static final Rotation2d kRedForward = Rotation2d.k180deg;

    // ================= CONSTRUCTOR =================
    public SwerveSubsystem(
        SwerveDrivetrainConstants constants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(constants, modules);

        SmartDashboard.putData("Field", field);

        // Vision trust (aggressive for testing)
        setVisionMeasurementStdDevs(
            VecBuilder.fill(0.1, 0.1, Math.toRadians(3))
        );

        registerTelemetry(telemetry::telemeterize);

        if (Utils.isSimulation()) startSim();
    }

    // ================= POSE + SPEEDS =================
    public Pose2d getPose() {
        return getState().Pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    public double getAngularVelocity() {
        return getState().Speeds.omegaRadiansPerSecond;
    }

    // ================= HEADING HOLD =================
    public void enableHeadingHold() {
        targetHeading = getPose().getRotation();
        headingHoldEnabled = true;
    }

    public void disableHeadingHold() {
        headingHoldEnabled = false;
    }

    public double getHeadingCorrection() {
        if (!headingHoldEnabled) return 0;

        double error =
            targetHeading.minus(getPose().getRotation()).getRadians();

        error = Math.atan2(Math.sin(error), Math.cos(error));
        return error * kP_heading;
    }

    // ================= DRIVE =================
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    public double getMaxSpeed() {
        return TunerConstants.kSpeedAt12Volts
            .in(edu.wpi.first.units.Units.MetersPerSecond);
    }

    // ================= VISION INJECTION =================
    private void updateVision() {

        var estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        if (estimate == null) return;
        if (estimate.tagCount <= 0) return;

        if (estimate.rawFiducials.length > 0 &&
            estimate.rawFiducials[0].ambiguity > MAX_VISION_AMBIGUITY)
            return;

        // 🔥 CRITICAL FIX:
        // Use Phoenix timebase, NOT Limelight timestamp
        addVisionMeasurement(
            estimate.pose,
            Utils.getCurrentTimeSeconds()
        );

        // Optional debug
        System.out.println("Vision Injected: " + estimate.pose);
    }

    // ================= PERIODIC =================
    @Override
    public void periodic() {

        if (!perspectiveApplied || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(
                    alliance == DriverStation.Alliance.Red
                        ? kRedForward
                        : kBlueForward
                );
                perspectiveApplied = true;
            });
        }

        field.setRobotPose(getPose());

        // Inject Limelight vision
        updateVision();
    }
}