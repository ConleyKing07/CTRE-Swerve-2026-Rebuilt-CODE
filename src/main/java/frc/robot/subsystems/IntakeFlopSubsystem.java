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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeFlopSubsystem extends SubsystemBase {

    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final SparkClosedLoopController armPID;

    private static final double STOW_POSITION = 0.0;
    private static final double DEPLOY_POSITION = 14;

    private static final double POSITION_TOLERANCE = 0.2;
    private static final double STOW_HOLD_OUTPUT = -0.15;

    private boolean clampActive = false;
    private boolean movingToStow = false;

    public IntakeFlopSubsystem() {

        armMotor = new SparkMax(22, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        armPID = armMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(30);

        config.closedLoop
            .p(0.6)
            .i(0.0)
            .d(0.2)
            .outputRange(-0.3, 0.3);

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
        clampActive = false;
        movingToStow = false;

        armPID.setSetpoint(
            DEPLOY_POSITION,
            ControlType.kPosition);
    }

    public void stow() {
        clampActive = false;
        movingToStow = true;

        armPID.setSetpoint(
            STOW_POSITION,
            ControlType.kPosition);
    }

    public void stop() {
        armMotor.stopMotor();
    }

    public boolean isDeployed() {
        return Math.abs(
            armEncoder.getPosition() - DEPLOY_POSITION
        ) < POSITION_TOLERANCE;
    }

    public boolean isStowed() {
        return Math.abs(
            armEncoder.getPosition() - STOW_POSITION
        ) < POSITION_TOLERANCE;
    }

    // =========================
    // PERIODIC
    // =========================

    @Override
    public void periodic() {

        // When moving to stow and we reach position
        if (movingToStow && isStowed()) {
            clampActive = true;
            movingToStow = false;
        }

        // Apply clamp hold ONLY after stow completed
        if (clampActive) {
            armMotor.set(STOW_HOLD_OUTPUT);
        }
    }
}