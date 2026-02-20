package frc.robot.commands.Autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.DriveAssistManager;

public class AutoAim extends Command {

    private final SwerveSubsystem swerve;
    private final DriveAssistManager driveAssist;

    private final ProfiledPIDController rotPID =
        new ProfiledPIDController(
            5.0, 0, 0,
            new TrapezoidProfile.Constraints(6, 12));

    private static final double AIM_TOLERANCE =
        Math.toRadians(1.0);

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
            swerve.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {

        double current =
            swerve.getPose().getRotation().getRadians();

        double target =
            driveAssist.getAimAngle().getRadians();

        double rot =
            rotPID.calculate(current, target);

        swerve.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rot));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotPID.getPositionError())
                < AIM_TOLERANCE;
    }
}
