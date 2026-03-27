package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeFlopSubsystem extends SubsystemBase {

    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final SparkClosedLoopController armPID;

    private static final double STOW_POSITION = 0.0;
    private static final double DEPLOY_POSITION = 11.5;

    private static final double POSITION_TOLERANCE = 0.5;
    private static final double STOW_HOLD_OUTPUT = -0.05;

    // =========================
    // TIMED JIMMY CONFIG
    // =========================
    private boolean jimmyActive = false;
    private double jimmyLow = 2.0;
    private double jimmyHigh = 11.5;
    private double jimmyPeriod = 0.5;
    private Timer jimmyTimer = new Timer();

    private boolean clampActive = false;
    private boolean movingToStow = false;

    public IntakeFlopSubsystem() {

        armMotor = new SparkMax(22, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        armPID = armMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        config.closedLoop
            .p(0.15)
            .i(0.0)
            .d(0.55)
            .outputRange(-0.325, 0.2);

        armMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        armEncoder.setPosition(0);
    }

    // =========================
    // COMMAND API
    // =========================

    public void deploy() {
        jimmyActive = false;
        clampActive = false;
        movingToStow = false;

        armPID.setSetpoint(DEPLOY_POSITION, ControlType.kPosition);
    }

    public void stow() {
        jimmyActive = false;
        clampActive = false;
        movingToStow = true;

        armPID.setSetpoint(STOW_POSITION, ControlType.kPosition);
    }

    public void stop() {
        jimmyActive = false;
        armMotor.stopMotor();
    }

    public void startJimmy() {
        jimmyActive = true;
        clampActive = false;
        movingToStow = false;

        jimmyTimer.reset();
        jimmyTimer.start();
    }

    public void stopJimmy() {
        jimmyActive = false;
        armMotor.stopMotor();
    }

    public boolean isDeployed() {
        return Math.abs(armEncoder.getPosition() - DEPLOY_POSITION) < POSITION_TOLERANCE;
    }

    public boolean isStowed() {
        return Math.abs(armEncoder.getPosition() - STOW_POSITION) < POSITION_TOLERANCE;
    }

    // =========================
    // PERIODIC
    // =========================

  @Override
public void periodic() {

    double position = armEncoder.getPosition();
    SmartDashboard.putNumber("Intake position", position);

    // =========================
    // TIMED JIMMY OSCILLATION
    // =========================
    if (jimmyActive) {
        double t = jimmyTimer.get() % jimmyPeriod;  // current time in cycle
        double halfPeriod = jimmyPeriod / 2.0;
        double target;

        // Start at high (11.5) → move to low first
        if (t < halfPeriod) {
            // moving high → low
            target = jimmyHigh - (jimmyHigh - jimmyLow) * (t / halfPeriod);
        } else {
            // moving low → high
            target = jimmyLow + (jimmyHigh - jimmyLow) * ((t - halfPeriod) / halfPeriod);
        }

        armPID.setSetpoint(target, ControlType.kPosition);
        return; // skip normal logic
    }

    // =========================
    // NORMAL LOGIC
    // =========================
    if (movingToStow && isStowed()) {
        clampActive = true;
        movingToStow = false;
    }

    if (clampActive) {
        armMotor.set(STOW_HOLD_OUTPUT);
    }
}
}