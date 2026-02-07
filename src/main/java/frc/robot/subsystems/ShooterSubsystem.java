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

    /* =============================
       Hardware
    ============================== */

    private final TalonFX master = new TalonFX(4);
    private final TalonFX follower = new TalonFX(5);

    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0);

    /* =============================
       Configuration
    ============================== */

    private static final double RPM_TOLERANCE = 75.0;

    private boolean armed = false;
    private double currentTargetRPM = 0;

    // Distance (meters) → RPM table
    private final TreeMap<Double, Double> rpmTable =
        new TreeMap<>();

    /* =============================
       Constructor
    ============================== */

    public ShooterSubsystem() {

        master.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);

        follower.setControl(
            new Follower(
                master.getDeviceID(),
                MotorAlignmentValue.Opposed
            )
        );

        /* ===== Initial RPM Table =====
           Replace with real data later
        */
        rpmTable.put(1.5, 2800.0);
        rpmTable.put(2.5, 3400.0);
        rpmTable.put(3.5, 4000.0);
        rpmTable.put(4.5, 4600.0);
        rpmTable.put(5.5, 5200.0);
    }

    /* =============================
       Arm / Safety
    ============================== */

    public void arm(){
        armed = true;
    }

    public void disarm(){
        armed = false;
        stop();
    }

    public boolean isArmed(){
        return armed;
    }

    /* =============================
       Control
    ============================== */

    public void setRPM(double rpm){

        if(!armed) return;

        currentTargetRPM = rpm;

        master.setControl(
            velocityRequest.withVelocity(
                rpm / 60.0
            )
        );
    }

    public void setRPMFromDistance(
        double distanceMeters
    ){
        setRPM(
            getRPMForDistance(distanceMeters)
        );
    }

    public void stop(){

        currentTargetRPM = 0;

        master.setControl(
            velocityRequest.withVelocity(0)
        );
    }

    /* =============================
       Feedback
    ============================== */

    public double getRPM(){
        return master.getVelocity()
            .getValueAsDouble() * 60.0;
    }

    public double getTargetRPM(){
        return currentTargetRPM;
    }

    public boolean atSpeed(){
        return Math.abs(
            getRPM() - currentTargetRPM
        ) < RPM_TOLERANCE;
    }

    public boolean readyToFire(){
        return armed && atSpeed();
    }

    /* =============================
       Distance → RPM Math
    ============================== */

    public double getRPMForDistance(
        double distanceMeters
    ){

        Map.Entry<Double, Double> lower =
            rpmTable.floorEntry(distanceMeters);

        Map.Entry<Double, Double> upper =
            rpmTable.ceilingEntry(distanceMeters);

        if(lower == null) return upper.getValue();
        if(upper == null) return lower.getValue();

        if(lower.getKey()
            .equals(upper.getKey()))
            return lower.getValue();

        double t =
            (distanceMeters - lower.getKey()) /
            (upper.getKey() - lower.getKey());

        return lower.getValue() +
            t * (upper.getValue()
            - lower.getValue());
    }

    /* =============================
       Live Tuning Support
    ============================== */

    public void addRPMPoint(
        double distanceMeters,
        double rpm
    ){
        rpmTable.put(distanceMeters, rpm);
    }
}
