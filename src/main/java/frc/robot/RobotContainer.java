package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.AutoShootCommand;
import frc.robot.vision.DriveAssistManager;

public class RobotContainer {

    private double MaxSpeed =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private double MaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake =
        new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.PointWheelsAt point =
        new SwerveRequest.PointWheelsAt();

    public final SwerveSubsystem drivetrain =
        TunerConstants.createDrivetrain();

    private final ShooterSubsystem shooter =
        new ShooterSubsystem();

    private final HopperSubsystem hopper =
        new HopperSubsystem();

    private final DriveAssistManager driveAssist =
        new DriveAssistManager(drivetrain);

    private final CommandXboxController driverXbox =
        new CommandXboxController(0);

    private final CommandXboxController scoringXbox =
        new CommandXboxController(1);

    private SendableChooser<String> autoChooser =
        new SendableChooser<>();

    public RobotContainer() {

        configureBindings();

        autoChooser.setDefaultOption(
            "1Algae1CoralCenter",
            "1Algae1CoralCenter"
        );

        autoChooser.addOption(
            "2PieceProcesor",
            "2PiecePR");

        autoChooser.addOption(
            "2PieceOposite",
            "2PieceOP");

        autoChooser.addOption(
            "PushBot1Algae1Coral",
            "PushBot1Algae1Coral"
        );

        Shuffleboard
            .getTab("Driver")
            .add(autoChooser);
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(
                        -driverXbox.getLeftY() * MaxSpeed)
                    .withVelocityY(
                        -driverXbox.getLeftX() * MaxSpeed)
                    .withRotationalRate(
                        -driverXbox.getRightX()
                        * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle)
                .ignoringDisable(true)
        );

        driverXbox.a()
            .whileTrue(
                drivetrain.applyRequest(() -> brake));

        driverXbox.b().whileTrue(
            drivetrain.applyRequest(() ->
                point.withModuleDirection(
                    new Rotation2d(
                        -driverXbox.getLeftY(),
                        -driverXbox.getLeftX()
                    )
                )
            )
        );

        // ⭐ AUTO SHOOT BUTTON
        scoringXbox.rightTrigger().whileTrue(
            new AutoShootCommand(
                drivetrain,
                shooter,
                hopper,
                driveAssist,
                () -> driverXbox.getLeftY(),
                () -> driverXbox.getLeftX()
            )
        );

        driverXbox.back().and(driverXbox.y())
            .whileTrue(
                drivetrain.sysIdDynamic(
                    Direction.kForward));

        driverXbox.back().and(driverXbox.x())
            .whileTrue(
                drivetrain.sysIdDynamic(
                    Direction.kReverse));

        driverXbox.start().and(driverXbox.y())
            .whileTrue(
                drivetrain.sysIdQuasistatic(
                    Direction.kForward));

        driverXbox.start().and(driverXbox.x())
            .whileTrue(
                drivetrain.sysIdQuasistatic(
                    Direction.kReverse));

        driverXbox.x().onTrue(
            drivetrain.runOnce(() ->
                drivetrain.seedFieldCentric())
        );
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(
            autoChooser.getSelected());
    }
}
