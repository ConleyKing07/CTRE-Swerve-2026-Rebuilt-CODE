package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class RightBank extends Command {

    // Mutable wrapper for lambda-safe variables
    private static class MutableDouble {
        public double value;
        public MutableDouble(double v) { value = v; }
    }

    private final SwerveSubsystem swerveDrive;

    private final SwerveRequest.RobotCentric driveRequest =
        new SwerveRequest.RobotCentric();

    private final PIDController turnPID;
    private final PIDController drivePID;
    private final PIDController strafePID;

    private static final double TARGET_X_RELATIVE = 0.2;
    private static final double TARGET_Y_RELATIVE = -0.16;

    private Translation2d lastKnownTargetPosition = null;
    private Double lastKnownRotationTarget = null;

    public RightBank(SwerveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        turnPID = new PIDController(0.10, 0, 0);
        drivePID = new PIDController(3.25, 0, 0);
        strafePID = new PIDController(3.25, 0, 0);
    }

    @Override
    public void execute() {

        double[] cameraposeTargetSpace =
            NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("camerapose_targetspace")
                .getDoubleArray(new double[6]);

        double tv =
            NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("tv")
                .getDouble(0);

        // Update target if tag is detected
        if (tv >= 1.0 && cameraposeTargetSpace.length >= 6) {
            lastKnownTargetPosition =
                new Translation2d(TARGET_X_RELATIVE, TARGET_Y_RELATIVE);
            lastKnownRotationTarget = 0.0;  // Face the tag
        }

        // If we have no last-known target, stop the robot
        if (lastKnownTargetPosition == null || lastKnownRotationTarget == null) {
            swerveDrive.applyRequest(
                () ->
                    driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
            );
            return;
        }

        // Current relative position
        Translation2d currentPosition =
            new Translation2d(cameraposeTargetSpace[0], cameraposeTargetSpace[2]);
        double currentHeading = cameraposeTargetSpace[4];

        // Use mutable wrappers
        MutableDouble driveOutput =
            new MutableDouble(
                drivePID.calculate(
                    currentPosition.getY(),
                    lastKnownTargetPosition.getY()
                )
            );

        MutableDouble strafeOutput =
            new MutableDouble(
                strafePID.calculate(
                    currentPosition.getX(),
                    lastKnownTargetPosition.getX()
                )
            );

        MutableDouble turnOutput =
            new MutableDouble(
                turnPID.calculate(currentHeading, lastKnownRotationTarget)
            );

        // Tolerance clean-up
        if (
            Math.abs(currentPosition.getY() - lastKnownTargetPosition.getY()) <
            0.01
        ) driveOutput.value = 0;

        if (
            Math.abs(currentPosition.getX() - lastKnownTargetPosition.getX()) <
            0.01
        ) strafeOutput.value = 0;

        if (
            Math.abs(currentHeading - lastKnownRotationTarget) < 0.01
        ) turnOutput.value = 0;

        // Apply updated drive request to CTRE swerve
        swerveDrive.applyRequest(
            () ->
                driveRequest
                    .withVelocityX(driveOutput.value)
                    .withVelocityY(-strafeOutput.value)
                    .withRotationalRate(-turnOutput.value)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // Continuous command
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.applyRequest(
            () ->
                driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
        );
        System.out.println("RightBank Ended");
    }
}
