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


public class IntakeFlopSubsystem {


// PID-controlled intake / hopper arm (mechanically linked)


private final SparkMax armMotor;
private final RelativeEncoder armEncoder;
private final SparkClosedLoopController armPID;


// Tunable setpoints (rotations)
private static final double STOW_POSITION = 0.0;
private static final double DEPLOY_POSITION = 4.25; // adjust to match linkage


public IntakeFlopSubsystem() {
armMotor = new SparkMax(2, MotorType.kBrushless);
armEncoder = armMotor.getEncoder();
armPID = armMotor.getClosedLoopController();


SparkMaxConfig config = new SparkMaxConfig();
config.idleMode(IdleMode.kBrake);
config.inverted(false);
config.smartCurrentLimit(40);


config.closedLoop
.p(2.2)
.i(0.0)
.d(0.15)
.outputRange(-0.6, 0.6);


armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


armEncoder.setPosition(0.0);
}


/** Deploys intake and expands hopper */
public void deploy() {
armPID.setSetpoint(DEPLOY_POSITION, ControlType.kPosition);
}


/** Stows intake and collapses hopper */
public void stow() {
armPID.setSetpoint(STOW_POSITION, ControlType.kPosition);
}


public boolean atDeploy() {
return Math.abs(armEncoder.getPosition() - DEPLOY_POSITION) < 0.1;
}


public boolean atStow() {
return Math.abs(armEncoder.getPosition() - STOW_POSITION) < 0.1;
}
}