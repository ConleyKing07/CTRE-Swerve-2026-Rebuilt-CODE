package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PreShooterSubsystem extends SubsystemBase {

    private final TalonFX master;
    private final TalonFX follower;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    // Relative RPM compared to shooter
    private static final double RPM_RATIO = 0.90;
    private static final double RPM_TOLERANCE = 100.0;

    public PreShooterSubsystem() {
        master = new TalonFX(6);
        follower = new TalonFX(7);

        master.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);

        // Follower spins same direction
        follower.setControl(new Follower(6, MotorAlignmentValue.Opposed));
    }

    /** Direct RPM set */
    public void setRPM(double rpm) {
        master.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }

    /** Follow shooter RPM */
    public void followShooter(double shooterRPM) {
        setRPM(shooterRPM * RPM_RATIO);
    }

    /** Stop pre-shooter */
    public void stop() {
        master.setControl(velocityRequest.withVelocity(0));
    }

    /** Current RPM */
    public double getRPM() {
        return master.getVelocity().getValueAsDouble() * 60.0;
    }

    /** At target check */
    public boolean atSetpoint(double targetRPM) {
        return Math.abs(getRPM() - targetRPM) < RPM_TOLERANCE;
    }
}
