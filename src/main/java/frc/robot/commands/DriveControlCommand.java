package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.DriveAssistManager;

public class DriveControlCommand extends Command {

    private final SwerveSubsystem swerve;
    private final DriveAssistManager driveAssist;

    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric();

    private final ProfiledPIDController rotPID =
        new ProfiledPIDController(
            5.0, 0, 0,
            new TrapezoidProfile.Constraints(6, 12)
        );

    private static final double ROT_RATE_THRESHOLD = 0.15;

    private boolean stable = false;

    public DriveControlCommand(
        SwerveSubsystem swerve,
        DriveAssistManager driveAssist,
        DoubleSupplier xInput,
        DoubleSupplier yInput
    ){
        this.swerve = swerve;
        this.driveAssist = driveAssist;
        this.xInput = xInput;
        this.yInput = yInput;

        rotPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        rotPID.reset(
            swerve.getPose()
                .getRotation()
                .getRadians()
        );
    }

    @Override
    public void execute(){

        Rotation2d targetAngle =
            driveAssist.getAimAngle();

        double currentHeading =
            swerve.getPose()
                .getRotation()
                .getRadians();

        double rotOutput =
            rotPID.calculate(
                currentHeading,
                targetAngle.getRadians()
            );

        double vx = -xInput.getAsDouble();
        double vy = -yInput.getAsDouble();

        swerve.setControl(
            drive.withVelocityX(vx)
                 .withVelocityY(vy)
                 .withRotationalRate(rotOutput)
        );

        stable = Math.abs(rotOutput) < ROT_RATE_THRESHOLD;
    }

    public boolean isStable(){
        return stable;
    }
}