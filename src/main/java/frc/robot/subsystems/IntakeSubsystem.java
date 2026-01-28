
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {


private final SparkMax intakeMotor;


public IntakeSubsystem() {
intakeMotor = new SparkMax(1, MotorType.kBrushless);


SparkMaxConfig config = new SparkMaxConfig();
config.smartCurrentLimit(40);
config.inverted(false);
config.idleMode(SparkMaxConfig.IdleMode.kBrake);
intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}


public void intakeIn() {
intakeMotor.set(1.0);
}


public void intakeOut() {
intakeMotor.set(-0.5);
}


public void stop() {
intakeMotor.stopMotor();
}
}