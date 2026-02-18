package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;
import java.util.TreeMap;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX master = new TalonFX(14);
    private final TalonFX follower = new TalonFX(15);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private static final double RPM_TOLERANCE = 25.0;

    private boolean armed = false;
    private double currentTargetRPM = 0;


    private final TreeMap<Double, Double> rpmTable = new TreeMap<>();

    public ShooterSubsystem() {

        master.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);

        var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 20;

        cfg.Slot0.kP = 0.75;
        cfg.Slot0.kV = 0.12;
        cfg.Slot0.kS = 0.05;

        master.getConfigurator().apply(cfg);

        follower.setControl(
            new Follower(
                master.getDeviceID(),
                MotorAlignmentValue.Opposed
            )
        );

        // RPM table
        rpmTable.put(1.75, 2400.0);
        rpmTable.put(2.0, 2500.0);
        rpmTable.put(2.5, 2700.0);
        rpmTable.put(3.0, 2900.0);
        rpmTable.put(3.5, 3100.0);
    }

    


    /** ==== Arm / Safety ==== */
    public void arm() {
        armed = true;
    }

    public void disarm() {
        armed = false;
        stop();
    }

    public boolean isArmed() {
        return armed && currentTargetRPM > 0 && atSpeed();
    }

    public boolean isSpinning() {
        return getRPM() > 500;
    }

    /** ==== Control ==== */
    public void setRPM(double rpm) {
    
        currentTargetRPM = rpm;

        master.setControl(
            velocityRequest.withVelocity(rpm / 60.0)
        );
    }

    public void setRPMFromDistance(double distanceMeters) {
        setRPM(getRPMForDistance(distanceMeters));
    }

    public void stop() {
        currentTargetRPM = 0;
        master.setControl(
            velocityRequest.withVelocity(0)
        );
    }

    /** ==== Feedback ==== */
    public double getRPM() {
        return master.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getTargetRPM() {
        return currentTargetRPM;
    }

    public boolean atSpeed() {
        return Math.abs(getRPM() - currentTargetRPM) < RPM_TOLERANCE;
    }

    public boolean readyToFire() {
        return armed && atSpeed();
    }

    /** ==== Distance → RPM Math ==== */
    public double getRPMForDistance(double distanceMeters) {
        Map.Entry<Double, Double> lower = rpmTable.floorEntry(distanceMeters);
        Map.Entry<Double, Double> upper = rpmTable.ceilingEntry(distanceMeters);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();
        if (lower.getKey().equals(upper.getKey())) return lower.getValue();

        double t = (distanceMeters - lower.getKey()) / (upper.getKey() - lower.getKey());
        return lower.getValue() + t * (upper.getValue() - lower.getValue());
    }

    /** ==== Live Tuning ==== */
    public void addRPMPoint(double distanceMeters, double rpm) {
        rpmTable.put(distanceMeters, rpm);
    }
}
