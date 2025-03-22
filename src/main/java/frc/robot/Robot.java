// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cscore.VideoSink;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  UsbCamera camera1;
  VideoSink server;
  NetworkTableEntry cameraSelection;
  Joystick joy1 = new Joystick(0);
  RobotContainer robotContainer;
  Command autonomousCommand;



  private Timer matchTimer;
  private double matchTimerRemaining;
  public Robot() {
    camera1 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    matchTimer = new Timer();
    matchTimerRemaining = 150;

    Logger.recordMetadata("ProjectName", "MyProject");

    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;

        // Logger.recordMetadata(("GitSHA"), Constants.BuildConstants.GitSHA);
    }

    Logger.start();

  }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
   robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  // @Override
  // public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    matchTimer.stop();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {  
  // matchTimer.reset();
  // matchTimer.start();
  //scheule command
  // robotContainer.autonInit();
  autonomousCommand = robotContainer.getAutonomousCommand();
  matchTimer.reset(); // Reset the timer when the match starts
  matchTimer.start(); // Start the timer

      // schedule the autonomous command (example)
      if (autonomousCommand != null) {
        autonomousCommand.schedule();
      }
 
  }
  
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
  
  

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  Logger.recordOutput("Time Remaining", matchTimerRemaining = 150 -(int) matchTimer.get());
  }
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


   // if (joy1.getTriggerPressed()) {
  //System.out.println("Setting camera 2");
  //cameraSelection.setString(camera2.getName());

    //   //server.setSource(camera2);
    // }else if (joy1.getTriggerReleased()) {
    //   System.out.println("Setting Camera 1");
    //   //server.setSource(camera1);
    //   cameraSelection.setString(camera1.getName());
    // }

    //RobotContainer.Test_controller();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}