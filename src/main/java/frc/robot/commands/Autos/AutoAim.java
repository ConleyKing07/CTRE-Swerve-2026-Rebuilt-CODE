package frc.robot.commands.Autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.DriveAssistManager;

public class AutoAim extends Command {

    private final SwerveSubsystem swerve;
    private final DriveAssistManager driveAssist;

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric();

    private final ProfiledPIDController rotPID =
            new ProfiledPIDController(
                    5.0, 0, 0,
                    new TrapezoidProfile.Constraints(6, 12));

    private static final double AIM_TOLERANCE =
            Math.toRadians(1.0);

    private static final double ROT_RATE_THRESHOLD =
            0.15; // rad/sec stability

    private boolean stable = false;

    public AutoAim(
            SwerveSubsystem swerve,
            DriveAssistManager driveAssist) {

        this.swerve = swerve;
        this.driveAssist = driveAssist;

        rotPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        rotPID.reset(
            swerve.getPose()
                .getRotation()
                .getRadians());
    }

    @Override
    public void execute() {

        Rotation2d targetAngle =
                driveAssist.getAimAngle();

        double currentHeading =
                swerve.getPose()
                .getRotation()
                .getRadians();

        double rotOutput =
                rotPID.calculate(
                        currentHeading,
                        targetAngle.getRadians());

        // No X/Y motion for pure auto-aim
        swerve.setControl(
                drive.withVelocityX(0)
                     .withVelocityY(0)
                     .withRotationalRate(rotOutput));

        stable =
            Math.abs(rotOutput)
                < ROT_RATE_THRESHOLD;
    }

    @Override
    public boolean isFinished() {
        return stable &&
               Math.abs(rotPID.getPositionError())
                   < AIM_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(
                drive.withVelocityX(0)
                     .withVelocityY(0)
                     .withRotationalRate(0));
    }
}