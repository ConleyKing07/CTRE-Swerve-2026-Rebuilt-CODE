package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class CompVisionManager {

    private final SwerveSubsystem swerve;

    private static final String LL = "limelight-shooter";

    private double lastVisionTime = 0;

    public CompVisionManager(SwerveSubsystem s){
        this.swerve = s;
    }

    public void update(){

        LimelightHelpers.setPipelineIndex(LL,1); // MegaTag1

        if(!LimelightHelpers.getTV(LL)) return;

        if(Math.abs(
            swerve.getAngularVelocity()
        ) > 2.5) return;

        Pose2d pose =
            LimelightHelpers
            .getBotPose2d_wpiBlue(LL);

        if(pose == null) return;

        double error =
            pose.getTranslation().getDistance(
                swerve.getPose().getTranslation()
            );

        if(error > 1.5) return;

        int tags =
            LimelightHelpers.getTargetCount(LL);

        double dist =
            LimelightHelpers
            .getBotPose3d_wpiBlue(LL)
            .getTranslation()
            .getNorm();

        Matrix<N3,N1> std =
            (tags>=2)?
                VecBuilder.fill(
                    .25+.06*dist,
                    .25+.06*dist,
                    Units.degreesToRadians(12))
            :
                VecBuilder.fill(
                    .7+.18*dist,
                    .7+.18*dist,
                    Units.degreesToRadians(35));

        double latency =
            LimelightHelpers.getLatency_Capture(LL)+
            LimelightHelpers.getLatency_Pipeline(LL);

        double ts =
            Timer.getFPGATimestamp()
            - latency/1000.0;

        swerve.addVisionMeasurement(
            pose, ts, std
        );

        lastVisionTime =
            Timer.getFPGATimestamp();
    }

    public boolean hasRecentVision(){
        return Timer.getFPGATimestamp()
            - lastVisionTime < 0.5;
    }
}