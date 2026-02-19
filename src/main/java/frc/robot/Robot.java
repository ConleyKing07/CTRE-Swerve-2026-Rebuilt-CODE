package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;
import org.opencv.core.*;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    startUsbCamera();
  }

  // ================= USB CAMERA SETUP =================
  private void startUsbCamera() {

    new Thread(() -> {

      // Start automatic capture (USB cam 0)
      UsbCamera camera = CameraServer.startAutomaticCapture(0);

      // Set max resolution (adjust if camera supports higher)
      camera.setResolution(192, 108);
      camera.setFPS(20);

      // Get video from camera
      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream =
          CameraServer.putVideo("FlippedCam", 192, 108);

      Mat mat = new Mat();

      while (!Thread.interrupted()) {

        if (cvSink.grabFrame(mat) == 0) {
          outputStream.notifyError(cvSink.getError());
          continue;
        }

        // Flip image 180° (flip both X and Y)
        Core.flip(mat, mat, -1);

        outputStream.putFrame(mat);
      }

    }).start();
  }

  // ================= ROBOT LOOP =================

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}