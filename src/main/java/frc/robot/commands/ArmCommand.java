package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.DriveAssistManager;

public class ArmCommand extends Command {

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final PreShooterSubsystem preshooter;
    private final DriveAssistManager driveAssist;

    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric();

    private final ProfiledPIDController rotPID =
            new ProfiledPIDController(
                    5.0,0,0,
                    new TrapezoidProfile.Constraints(6,12));

    private static final double AIM_TOLERANCE =
            Math.toRadians(1.0);

    private static final double ROT_RATE_THRESHOLD =
            0.15; // rad/sec stability

    private boolean ready=false;
    private boolean stable=false;

    public ArmCommand(
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            PreShooterSubsystem preshooter,
            DriveAssistManager driveAssist,
            DoubleSupplier xInput,
            DoubleSupplier yInput){

        this.swerve=swerve;
        this.shooter=shooter;
        this.preshooter=preshooter;
        this.driveAssist=driveAssist;
        this.xInput=xInput;
        this.yInput=yInput;

        rotPID.enableContinuousInput(-Math.PI,Math.PI);

        addRequirements(swerve,shooter,preshooter);
    }

    @Override
    public void initialize(){

        shooter.arm();

        rotPID.reset(
            swerve.getPose()
            .getRotation()
            .getRadians());
    }

    @Override
    public void execute(){

        Rotation2d targetAngle =
                driveAssist.getAimAngle();

        double currentHeading =
                swerve.getPose()
                .getRotation().getRadians();

        double rotOutput =
                rotPID.calculate(
                        currentHeading,
                        targetAngle.getRadians());

        double distance =
                driveAssist.getDistanceToTarget();

        shooter.setRPMFromDistance(distance);

        preshooter.followShooter(
            shooter.getTargetRPM());

        double vx = -xInput.getAsDouble();
        double vy = -yInput.getAsDouble();

        swerve.setControl(
                drive.withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(rotOutput));

        stable =
            Math.abs(rotOutput)
            < ROT_RATE_THRESHOLD;

        ready =
            shooter.readyToFire()
            && preshooter.atSpeed()
            && stable
            && Math.abs(rotPID.getPositionError())
                < AIM_TOLERANCE;
    }

    public boolean isReady(){
        return ready;
    }

    public boolean isStable(){
        return stable;
    }

    @Override
    public void end(boolean interrupted){

        shooter.disarm();
        preshooter.stop();
    }
}
