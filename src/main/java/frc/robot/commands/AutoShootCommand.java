package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.vision.DriveAssistManager;

import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.function.DoubleSupplier;

public class AutoShootCommand extends Command {

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final DriveAssistManager driveAssist;

    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private final ProfiledPIDController rotPID =
            new ProfiledPIDController(
                    5.0, 0, 0,
                    new TrapezoidProfile.Constraints(6, 12)
            );

    private static final double AIM_TOLERANCE = Math.toRadians(2.0);

    public AutoShootCommand(
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            DriveAssistManager driveAssist,
            DoubleSupplier xInput,
            DoubleSupplier yInput
    ) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.hopper = hopper;
        this.driveAssist = driveAssist;
        this.xInput = xInput;
        this.yInput = yInput;

        rotPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve, shooter, hopper);
    }

    @Override
    public void initialize() {
        shooter.arm();
    }

    @Override
    public void execute() {

        // 1️⃣ Compute target angle
        Rotation2d targetAngle = driveAssist.getAimAngle();
        double currentHeading = swerve.getPose().getRotation().getRadians();
        double rotOutput = rotPID.calculate(currentHeading, targetAngle.getRadians());

        // 2️⃣ Compute target RPM based on distance
        double distance = driveAssist.getDistanceToTarget();
        shooter.setRPMFromDistance(distance);

        // 3️⃣ Drive control (can combine joystick override if needed)
        double vx = -xInput.getAsDouble();
        double vy = -yInput.getAsDouble();

        swerve.setControl(
                drive.withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(rotOutput)
        );

        // 4️⃣ Hopper management: only feed if shooter is ready
        if (shooter.readyToFire() && Math.abs(rotPID.getPositionError()) < AIM_TOLERANCE) {
            hopper.feed();
        } else {
            hopper.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disarm();
        hopper.stop();
    }
}
