package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.vision.DriveAssistManager;

public class RobotContainer {

    // ---------------- Speed limits ----------------
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);

    // ---------------- Subsystems ----------------
    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final PreShooterSubsystem preshooter = new PreShooterSubsystem();
    private final HopperSubsystem hopper = new HopperSubsystem();
    private final IntakeFlopSubsystem intakeFlop = new IntakeFlopSubsystem();
    private final DriveAssistManager driveAssist = new DriveAssistManager(drivetrain);

    // ---------------- Controllers ----------------
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController scoringXbox = new CommandXboxController(1);

    // ---------------- Commands ----------------
    private final ArmCommand armShoot = new ArmCommand(
            drivetrain,
            shooter,
            preshooter,
            driveAssist,
            () -> driverXbox.getLeftY(),
            () -> driverXbox.getLeftX()
    );

    private final FireCommand fire = new FireCommand(hopper);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    // ---------------- Drive requests ----------------
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // ---------------- Speed boost ----------------
    private boolean speedBoost = false;
    private static final double kNormalSpeedScale = 0.6;
    private static final double kBoostSpeedScale = 1.0;

    // ---------------- Constructor ----------------
    public RobotContainer() {
        configureBindings();

        autoChooser.setDefaultOption("NewAuto", "NewAuto");
        autoChooser.addOption("2PieceProcesor", "2PiecePR");
        autoChooser.addOption("2PieceOposite", "2PieceOP");
        autoChooser.addOption("PushBot1Algae1Coral", "PushBot1Algae1Coral");

        Shuffleboard.getTab("Driver").add(autoChooser);
    }

    // ---------------- Bindings ----------------
    private void configureBindings() {

        // ---------------- Speed boost toggle ----------------
        driverXbox.rightBumper().whileTrue(new InstantCommand(() -> speedBoost = true))
                                .onFalse(new InstantCommand(() -> speedBoost = false));

        // ---------------- Default drive command ----------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double speedScale = speedBoost ? kBoostSpeedScale : kNormalSpeedScale;

                double vx = -driverXbox.getLeftY() * MaxSpeed * speedScale;
                double vy = -driverXbox.getLeftX() * MaxSpeed * speedScale;
                double rot = -driverXbox.getRightX() * MaxAngularRate * speedScale;

                // Heading-hold when right stick is near zero
                if (Math.abs(driverXbox.getRightX()) < 0.05) {
                    if (!drivetrain.headingHoldEnabled) drivetrain.enableHeadingHold();
                    double error = drivetrain.getPose().getRotation()
                                    .minus(drivetrain.targetHeading)
                                    .getRadians();
                    rot += SwerveSubsystem.kP_heading * error;
                } else {
                    drivetrain.disableHeadingHold();
                }

                return drive.withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(rot);
            })
        );

        final var idle = new SwerveRequest.Idle();

        // ---------------- Disabled idle ----------------
        RobotModeTriggers.disabled()
            .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // ---------------- Intake (Analog Trigger Control) ----------------
intake.setDefaultCommand(
    intake.run(() -> {

        double intakeAxis = scoringXbox.getRightTriggerAxis();
        double outtakeAxis = scoringXbox.getLeftTriggerAxis();

        // Deadband
        if (intakeAxis < 0.1) intakeAxis = 0;
        if (outtakeAxis < 0.1) outtakeAxis = 0;

        // Right trigger = intake forward
        // Left trigger = reverse
        double speed = intakeAxis - outtakeAxis;

        intake.intakeRun(speed);
    })
);

scoringXbox.a().onTrue(new IntakeDeploy(intakeFlop));
scoringXbox.b().onTrue(new IntakeRetract(intakeFlop));


        // ---------------- Shooting ----------------
        scoringXbox.leftBumper().whileTrue(armShoot);
        scoringXbox.rightBumper().whileTrue(fire);

        // ---------------- Driver utilities ----------------
        driverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(
                new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX())
            )
        ));
        driverXbox.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    
    }

    // ---------------- Autonomous ----------------
    public Command getAutonomousCommand() {
        return new PathPlannerAuto(autoChooser.getSelected());
    }
}
