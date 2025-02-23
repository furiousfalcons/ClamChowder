// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Down_Arm;
import frc.robot.commands.Elevator_Up;
import frc.robot.commands.Elevator_Down;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.Up_Arm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.InTakeOutPut;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private Arm arm = new Arm();
  private InTakeOutPut intakeShooter = new InTakeOutPut();
  private Elevator climb = new Elevator();

  private final IntakeIn intake = new IntakeIn(intakeShooter);
  private final IntakeOut output = new IntakeOut(intakeShooter);
  private final Elevator_Up elevator_Up = new Elevator_Up(climb);
  private final Elevator_Down elevator_Down = new Elevator_Down(climb);
  private final Up_Arm armUp = new Up_Arm(arm);
  private final Down_Arm armDown = new Down_Arm(arm);

 


  // The driver's controller
  static XboxController m_driverController1 = new XboxController(OIConstants.kDriverControllerPort);
  static XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort1);
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      m_robotDrive.setDefaultCommand(
          
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController1.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController1.getLeftX(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController1.getRightX(), OIConstants.kDriveDeadband),
                  true, true),
              m_robotDrive));
              new JoystickButton(m_driverController1, Button.kR1.value)
              .whileTrue(new RunCommand( () -> m_robotDrive.setX(), m_robotDrive));
    }
  
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
      JoystickButton armUpButton = new JoystickButton(m_driverController1, 4);
      JoystickButton armDownButton = new JoystickButton(m_driverController1, 1);
      JoystickButton inTakeButton = new JoystickButton(m_driverController1, 5);
      JoystickButton shootButton = new JoystickButton(m_driverController1, 6);
      JoystickButton elevatorUpButton = new JoystickButton(m_driverController1, 2);
      JoystickButton elevatorDownButton = new JoystickButton(m_driverController1, 3);
      // armUpButton.whileTrue(armUp);
      // armDownButton.whileTrue(armDown);
      elevatorUpButton.toggleOnTrue(elevator_Up);
      elevatorDownButton.toggleOnTrue(elevator_Down);
      // inTakeButton .whileTrue(intake);
      // shootButton.whileTrue(output);


    }
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return 
     */
    public Object getAutonomousCommand() {
      // Create config for trajectory
     /*  TrajectoryConfig config = new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics);
  
      // An example trajectory to follow. All units in meters.
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          config);
  
      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,
  
          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);
  
      // Reset odometry to the starting pose of the trajectory.
      m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
   
      // Run path following command, then stop at the end.
      return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
  */  return null;}
    public void logTheBits() {
      // DriverStation.reportError(arm.getMeasurement()  +"", false);
    }
  
    /*public static void Test_controller() {
      double potato = m_driverController.getLeftX();
      int i = 1;
      while (i < 5) {
        System.out.println(potato);
        i++;
      }*/
  }
