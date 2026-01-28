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

    private final TalonFX master;
    private final TalonFX follower;

    // Velocity PID request
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private static final double RPM_TOLERANCE = 75.0;

    // Distance → RPM table (meters)
    private final TreeMap<Double, Double> rpmTable = new TreeMap<>();

    public ShooterSubsystem() {
        master = new TalonFX(4);
        follower = new TalonFX(5);

        master.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);

        // Follower mirrors master (opposing)
       follower.setControl(new Follower(4, MotorAlignmentValue.Opposed)); 
        // Example RPM table — fill in with your real data
        rpmTable.put(0.5, 3800.0);
        rpmTable.put(1.0, 4200.0);
        rpmTable.put(1.5, 4800.0);
        rpmTable.put(2.0, 5200.0);
        rpmTable.put(2.5, 5200.0);
        rpmTable.put(3.0, 5200.0);
        rpmTable.put(3.5, 5200.0);
        rpmTable.put(4.0, 5200.0);
        rpmTable.put(4.5, 5200.0);
        rpmTable.put(5.0, 5200.0);
        rpmTable.put(5.5, 5200.0);
        rpmTable.put(6.0, 5200.0);
    }

    /** Set shooter RPM directly */
    public void setRPM(double rpm) {
        master.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }

    /** Stop shooter */
    public void stop() {
        master.setControl(velocityRequest.withVelocity(0));
    }

    /** Current RPM from master */
    public double getRPM() {
        return master.getVelocity().getValueAsDouble() * 60.0;
    }

    /** Shooter at target RPM */
    public boolean atSetpoint(double targetRPM) {
        return Math.abs(getRPM() - targetRPM) < RPM_TOLERANCE;
    }

    /** Get RPM from distance using interpolation */
    public double getRPMForDistance(double distanceMeters) {
        Map.Entry<Double, Double> lower = rpmTable.floorEntry(distanceMeters);
        Map.Entry<Double, Double> upper = rpmTable.ceilingEntry(distanceMeters);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();
        if (lower.getKey().equals(upper.getKey())) return lower.getValue();

        // Linear interpolation
        double distDiff = upper.getKey() - lower.getKey();
        double rpmDiff = upper.getValue() - lower.getValue();
        double factor = (distanceMeters - lower.getKey()) / distDiff;
        return lower.getValue() + factor * rpmDiff;
    }

    /** Set shooter RPM automatically based on Limelight distance */
    public void setRPMFromDistance(double distanceMeters) {
        setRPM(getRPMForDistance(distanceMeters));
    }
}
