package frc.robot.commands.Autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.DriveAssistManager;

public class AutoAimCommand extends Command {

    private final SwerveSubsystem swerve;
    private final DriveAssistManager driveAssist;

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric();

    public AutoAimCommand(
        SwerveSubsystem swerve,
        DriveAssistManager driveAssist
    ){
        this.swerve = swerve;
        this.driveAssist = driveAssist;

        addRequirements(swerve);
    }

    @Override
    public void execute(){

        double rot =
            driveAssist.getAimAngle()
            .minus(swerve.getPose().getRotation())
            .getRadians() * 4.0;

        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(rot)
        );
    }
}