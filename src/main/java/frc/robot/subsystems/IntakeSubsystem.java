package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor;

   
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(21, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40);
        config.inverted(true);
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

  

   

  
    /** Run intake in */
    public void intakeRun(double speed){
        intakeMotor.set(speed);
    }


    /** Stop intake */
    public void stop() {
        intakeMotor.set(0);
    }
}
