package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Telemetry;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.vision.DriveAssistManager;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.*;

public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {

    // ================= DRIVE ASSIST =================
    private final DriveAssistManager driveAssist;

    // ================= HEADING HOLD =================
    public Rotation2d targetHeading = Rotation2d.kZero;
    public boolean headingHoldEnabled = false;
    public static final double kP_heading = 0.08;

    // ================= LIMELIGHT =================
    private static final String LIMELIGHT_NAME = "limelight-shooter";
    private static final double MAX_VISION_AMBIGUITY = 0.7;
    private double lastVisionTimestamp = -1;

    // ================= FIELD VIS =================
    private final Field2d field = new Field2d();

    private final Telemetry telemetry =
        new Telemetry(
            TunerConstants.kSpeedAt12Volts
                .in(edu.wpi.first.units.Units.MetersPerSecond)
        );

    // ================= ALLIANCE PERSPECTIVE =================
    private boolean perspectiveApplied = false;
    private static final Rotation2d kBlueForward = Rotation2d.kZero;
    private static final Rotation2d kRedForward = Rotation2d.k180deg;

    // ================= SIM =================
    private Notifier simNotifier;
    private double lastSimTime;

    private void startSim() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            double now = Utils.getCurrentTimeSeconds();
            double dt = now - lastSimTime;
            lastSimTime = now;
            updateSimState(dt, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(0.02);
    }

    // ================= CONSTRUCTOR =================
    public SwerveSubsystem(
        SwerveDrivetrainConstants constants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(constants, modules);

        driveAssist = new DriveAssistManager(this);

        for (var module : getModules()) {
            module.getDriveMotor().getPosition().setUpdateFrequency(150);
            module.getDriveMotor().getVelocity().setUpdateFrequency(150);

            module.getSteerMotor().getPosition().setUpdateFrequency(150);
            module.getSteerMotor().getVelocity().setUpdateFrequency(150);

            getPigeon2().getYaw().setUpdateFrequency(150);
            getPigeon2().getAngularVelocityZWorld().setUpdateFrequency(150);
        }

        SmartDashboard.putData("Field", field);

        setVisionMeasurementStdDevs(
            VecBuilder.fill(0.1, 0.1, Math.toRadians(3))
        );

        registerTelemetry(telemetry::telemeterize);

        if (Utils.isSimulation()) startSim();

        configureAutoBuilder();
    }

    // ================= POSE =================
    public Pose2d getPose() {
        return getState().Pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    public double getAngularVelocity() {
        return getState().Speeds.omegaRadiansPerSecond;
    }

    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    public double getMaxSpeed() {
        return TunerConstants.kSpeedAt12Volts
            .in(edu.wpi.first.units.Units.MetersPerSecond);
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

    private void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    // ================= PATHPLANNER =================
    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(0.08, 0.0, 0.0),
                    new PIDConstants(0.00, 0.0, 0.0)
                ),
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance()
                        .map(a -> a == DriverStation.Alliance.Red)
                        .orElse(false),
                this
            );
        } catch (Exception e) {
            System.out.println("PathPlanner config failed!");
            e.printStackTrace();
        }
    }

    // ================= LIMELIGHT =================
    private void updateVision() {

        var estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        if (estimate == null) return;
        if (estimate.tagCount <= 0) return;

        if (estimate.rawFiducials.length > 0 &&
            estimate.rawFiducials[0].ambiguity > MAX_VISION_AMBIGUITY)
            return;

        if (estimate.timestampSeconds == lastVisionTimestamp) return;
        lastVisionTimestamp = estimate.timestampSeconds;

        double distance = estimate.avgTagDist;

        double xyStdDev = 0.3 + (distance * 0.1);
        double thetaStdDev = Math.toRadians(5 + distance * 2);

        setVisionMeasurementStdDevs(
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
        );

        addVisionMeasurement(
            estimate.pose,
            Utils.getCurrentTimeSeconds()
        );
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

        updateVision();

        // 🔥 Drive assist telemetry
        driveAssist.publishTelemetry();
    }
}
