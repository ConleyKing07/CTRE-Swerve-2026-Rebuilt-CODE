package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoL extends Command {

    private final SwerveSubsystem swerveDrive;

    // CTRE swerve request for driving with chassis speeds
    private final SwerveRequest.RobotCentric driveRequest =
        new SwerveRequest.RobotCentric();

    private final PIDController turnPID;
    private final PIDController drivePID;
    private final PIDController strafePID;

    private static final double TARGET_X_RELATIVE = -0.01;
    private static final double TARGET_Y_RELATIVE = -0.15;

    private Translation2d lastKnownTargetPosition = null;
    private Double lastKnownRotationTarget = null;
    private Translation2d currentPosition = null;
    private double currentHeading = 0.0;

    public AutoL(SwerveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        turnPID = new PIDController(0.08, 0, 0);
        drivePID = new PIDController(3.4, 0, 0);
        strafePID = new PIDController(3.25, 0, 0);
    }

    @Override
    public void execute() {

        double[] cameraposeTargetSpace = NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("camerapose_targetspace")
            .getDoubleArray(new double[6]);

        double tv = NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tv")
            .getDouble(0);

        if (tv >= 1.0 && cameraposeTargetSpace.length >= 6) {

            double cameraXRelative = cameraposeTargetSpace[0];
            double cameraYRelative = cameraposeTargetSpace[2];
            double cameraYawRelative = cameraposeTargetSpace[4];

            lastKnownTargetPosition =
                new Translation2d(TARGET_X_RELATIVE, TARGET_Y_RELATIVE);

            lastKnownRotationTarget = 0.0;

            currentPosition =
                new Translation2d(cameraXRelative, cameraYRelative);
            currentHeading = cameraYawRelative;
        }

        if (lastKnownTargetPosition == null || lastKnownRotationTarget == null) {
            // Replace stop drive with CTRE no-movement request
            swerveDrive.applyRequest(() -> driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double driveOutput =
            drivePID.calculate(currentPosition.getY(), lastKnownTargetPosition.getY());

        double strafeOutput =
            strafePID.calculate(currentPosition.getX(), lastKnownTargetPosition.getX());

        double turnOutput =
            turnPID.calculate(currentHeading, lastKnownRotationTarget);

        // Build CTRE swerve request (robot-centric)
        swerveDrive.applyRequest(
            () ->
                driveRequest
                    .withVelocityX(driveOutput)
                    .withVelocityY(-strafeOutput)
                    .withRotationalRate(-turnOutput)
        );
    }

    @Override
    public boolean isFinished() {

        if (lastKnownTargetPosition == null || currentPosition == null) {
            return false;
        }

        boolean atTargetX =
            Math.abs(currentPosition.getX() - lastKnownTargetPosition.getX()) < 0.1;

        boolean atTargetY =
            Math.abs(currentPosition.getY() - lastKnownTargetPosition.getY()) < 0.1;

        boolean atTargetHeading =
            Math.abs(currentHeading - lastKnownRotationTarget) < 3.0;

        return atTargetX && atTargetY && atTargetHeading;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.applyRequest(
            () -> driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
    }
}
