package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {

    /* ================= SYSID ================= */
    private final SwerveRequest.SysIdSwerveTranslation sysidReq = new SwerveRequest.SysIdSwerveTranslation();
    private final SysIdRoutine sysid = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> setControl(sysidReq.withVolts(volts)),
            null,
            this
        )
    );

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return sysid.dynamic(dir);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return sysid.quasistatic(dir);
    }

    /* ================= SIMULATION ================= */
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
        simNotifier.startPeriodic(0.005);
    }

    /* ================= OPERATOR PERSPECTIVE ================= */
    private boolean perspectiveApplied = false;
    private static final Rotation2d kBlueForward = Rotation2d.kZero;
    private static final Rotation2d kRedForward = Rotation2d.k180deg;

    /* ================= CONSTRUCTOR ================= */
    public SwerveSubsystem(SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(constants, modules);

        setVisionMeasurementStdDevs(
            VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(6))
        );

        if (Utils.isSimulation()) startSim();
    }

    /* ================= CORE METHODS ================= */

    /** Current robot pose */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /** Current robot chassis speeds */
    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    /** Angular velocity in radians/sec */
    public double getAngularVelocity() {
        return getState().Speeds.omegaRadiansPerSecond;
    }

    /** Apply a swerve request as a command */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    /** Reset the robot pose for field-centric driving */
    public void resetPose(Pose2d pose) {
        seedFieldCentric(pose.getRotation());
        addVisionMeasurement(pose, Timer.getFPGATimestamp(),
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1))
        );
    }

    /** Maximum linear speed */
     public double getMaxSpeed(){
        return TunerConstants
            .kSpeedAt12Volts
            .in(edu.wpi.first.units.Units.MetersPerSecond);
    }

    /* ================= PERIODIC ================= */
    @Override
    public void periodic() {
        if (!perspectiveApplied || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(
                    alliance == DriverStation.Alliance.Red ? kRedForward : kBlueForward
                );
                perspectiveApplied = true;
            });
        }
    }
}
