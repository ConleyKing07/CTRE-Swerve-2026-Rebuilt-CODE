package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

    private final SparkMax hopperMotor;
   

    // Default speeds (adjust to your robot)
    private static final double FEED_SPEED = 1.0;  // toward shooter
    private static final double REVERSE_SPEED = -0.5; // out / clear jam


    public HopperSubsystem() {
       
        final SparkMaxConfig config = new SparkMaxConfig();
        hopperMotor = new SparkMax(23, MotorType.kBrushless);
        config.inverted(true);
        config.smartCurrentLimit(40);
        hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    

    /** Feed balls toward shooter at default speed */
    public void feed() {
        hopperMotor.set(FEED_SPEED);
    }

    /** Reverse hopper to clear jams */
    public void reverse() {
        hopperMotor.set(REVERSE_SPEED);
    }

    /** Stop hopper */
    public void stop() {
        hopperMotor.set(0);
    }

    /** Feed with custom speed (-1.0 to 1.0) */
    public void feed(double speed) {
        hopperMotor.set(speed);
    }

    
    }

