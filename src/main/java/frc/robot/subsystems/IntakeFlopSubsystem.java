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

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeFlopSubsystem extends SubsystemBase {

    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final SparkClosedLoopController armPID;

    private final CANrange hopperSensor =
        new CANrange(10);

    private static final double STOW_POSITION = 0.0;
    private static final double DEPLOY_POSITION = 4.25;

    /* ==== TUNE THIS ==== */
    private static final double Full_DISTANCE_METERS =
        0.25; // when hopper empty

    private static final double POST_FOLD_DELAY =
        1.5;

    private boolean autoFoldActive=false;
    private double foldTime=0;
    private boolean manualDeploy=false;

    public IntakeFlopSubsystem(){

        armMotor=new SparkMax(2,MotorType.kBrushless);
        armEncoder=armMotor.getEncoder();
        armPID=armMotor.getClosedLoopController();

        SparkMaxConfig config=new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        config.closedLoop
            .p(2.2)
            .i(0)
            .d(0.15)
            .outputRange(-0.6,0.6);

        armMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        armEncoder.setPosition(0);
    }

    public void requestDeploy(boolean deploy){
        manualDeploy=deploy;
    }

    private void deploy(){
        armPID.setSetpoint(
            DEPLOY_POSITION,
            ControlType.kPosition);
    }

    private void stow(){
        armPID.setSetpoint(
            STOW_POSITION,
            ControlType.kPosition);
    }

    private double getDistance(){
        return hopperSensor
            .getDistance()
            .getValueAsDouble();
    }

    @Override
    public void periodic(){

        double dist=getDistance();

        if(dist>Full_DISTANCE_METERS
            && !autoFoldActive){

            autoFoldActive=true;
            foldTime=Timer.getFPGATimestamp();
            stow();
        }

        if(autoFoldActive){

            if(Timer.getFPGATimestamp()
                -foldTime>POST_FOLD_DELAY){

                autoFoldActive=false;
            }

            return;
        }

        if(manualDeploy){
            deploy();
        }
    }
}
