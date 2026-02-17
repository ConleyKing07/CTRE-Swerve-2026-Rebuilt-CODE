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

    private static final double RPM_RATIO = 1.15;
    private static final double RPM_TOLERANCE = 50.0;

    private double targetRPM = 0;

    public PreShooterSubsystem() {

        master = new TalonFX(16);
        follower = new TalonFX(17);

        master.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);

 var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 20;

        cfg.Slot0.kP = 0.12;
        cfg.Slot0.kV = 0.12;
        cfg.Slot0.kS = 0.05;

        master.getConfigurator().apply(cfg);

        follower.setControl(
            new Follower(
                master.getDeviceID(),
                MotorAlignmentValue.Opposed
            )
        );
    }

   
   

    
    /** Direct RPM set */
    public void setRPM(double rpm) {

        targetRPM = rpm;

        master.setControl(
            velocityRequest.withVelocity(rpm / 60.0)
        );
    }

    /** Follow shooter RPM */
    public void followShooter(double targetshooterRPM) {
        setRPM(-targetshooterRPM * RPM_RATIO);
    }

    public void stop() {
        targetRPM = 0;
        master.setControl(
            velocityRequest.withVelocity(0)
        );
    }

    public double getRPM() {
        return master.getVelocity().getValueAsDouble() * 60.0;
    }

    public boolean atSpeed() {
        return Math.abs(getRPM() - targetRPM) < RPM_TOLERANCE;
    }

    public double getTargetRPM() {
        return targetRPM;
    }
}
