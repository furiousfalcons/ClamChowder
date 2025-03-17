// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbON;
import frc.robot.commands.ClimbStop;
import frc.robot.commands.Down_Arm;
import frc.robot.commands.Elevator_Up;
import frc.robot.commands.Elevator_Down;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.MasterCommands;
import frc.robot.commands.Stop_Arm;
import frc.robot.commands.Stop_Intake;
import frc.robot.commands.Up_Arm;
import frc.robot.commands.climbDown;
import frc.robot.subsystems.Arm;
import frc.robot.commands.ClimbON;
import frc.robot.commands.ClimbStop;
import frc.robot.subsystems.Climb;
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

  private final InTakeOutPut intake;
  private final Arm arms;
  private final Elevator elevator;
  // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private InTakeOutPut intakeShooter = new InTakeOutPut();
  private Elevator climb = new Elevator();
  private Arm arm = new Arm();
   private Climb goClimb = new Climb();
  private final IntakeIn inTake = new IntakeIn(intakeShooter);
  private final IntakeOut shoot = new IntakeOut(intakeShooter);
  private final climbDown climb_p = new climbDown(goClimb);
  private final ClimbON climb_r = new ClimbON(goClimb);
  private final ClimbStop climb_f = new ClimbStop(goClimb);
  private final Stop_Intake stopIntake = new Stop_Intake(intakeShooter);
   private final Up_Arm armUp = new Up_Arm(arm);
  private final Down_Arm armDown = new Down_Arm(arm);
  private final Stop_Arm armStop = new Stop_Arm(arm);
 
  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  static XboxController m_driverController1 = new XboxController(OIConstants.kDriverControllerPort);
  static XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort1);
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

intake = new InTakeOutPut();
elevator = new Elevator();
arms = new Arm();

NamedCommands.registerCommand("Stop Intake", MasterCommands.stopIntake(intake));
NamedCommands.registerCommand("Run Intake", MasterCommands.runIntake(intake));
NamedCommands.registerCommand("Run Output", MasterCommands.runOutPut(intake));

// NamedCommands.registerCommand("Set Arm Position", MasterCommands.setArmPosition(arms, elevator));

// NamedCommands.registerCommand(
//             "Set Elevator L1 C",
//             MasterCommands.setElevatorPosition(
//                 arms,
//                 elevator,
//                 ElevatorConstants.L1_CORAL,
//                 ArmConstants.ARM_REST_POSITION));

//                 NamedCommands.registerCommand(
//             "Set Elevator L2 C",
//             MasterCommands.setElevatorPosition(
//                 arms,
//                 elevator,
//                 ElevatorConstants.L2_CORAL,
//                 ArmConstants.ARM_CORAL_POSITION));

//                  NamedCommands.registerCommand(
//                   "Set Elevator L3 C",
//                   MasterCommands.setElevatorPosition(
//                       arms,
//                       elevator,
//                       ElevatorConstants.L3_CORAL,
//                       ArmConstants.ARM_CORAL_POSITION));

//                       NamedCommands.registerCommand(
//                   "Set Elevator L2 A",
//                   MasterCommands.setElevatorPosition(
//                       arms,
//                       elevator,
//                       ElevatorConstants.L2_ALGEA,
//                       ArmConstants.ARM_ALGEA_POSITION));

//                       NamedCommands.registerCommand(
//                   "Set Elevator L3 A",
//                   MasterCommands.setElevatorPosition(
//                       arms,
//                       elevator,
//                       ElevatorConstants.L3_ALGEA,
//                       ArmConstants.ARM_ALGEA_POSITION));

//                       NamedCommands.registerCommand(
//                   "Set Elevator Amp A",
//                   MasterCommands.setElevatorPosition(
//                       arms,
//                       elevator,
//                       ElevatorConstants.AMP_ALGEA,
//                       ArmConstants.ARM_REST_POSITION));


                      

      // boolean isCompetition = true;
      // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      //   (stream) -> isCompetition
      //   ? stream.filter(auto -> auto.getName().startsWith("comp"))
      //   : stream
      // );
   
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
              new JoystickButton(m_driverController1, Button.kR3.value)
              .whileTrue(new RunCommand( () -> m_robotDrive.setX(), m_robotDrive));

      climb.setDefaultCommand(
        new RunCommand(
          () -> climb.toggleElevator(
            -MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(), 0.05),
            -MathUtil.applyDeadband(m_driverController1.getRightTriggerAxis(), 0.05)
          ), 
        climb));
           autoChooser = AutoBuilder.buildAutoChooser();
           SmartDashboard.putData("Auto Chooser", autoChooser);



        // arm.setDefaultCommand(
        //   new RunCommand(
        //     () -> arm.
        //   )
        // );
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

      // SmartDashboard.putData("Straight Auto.auto", new PathPlannerAuto("Testing"));

      JoystickButton shootButton = new JoystickButton(m_driverController1, 3);
      JoystickButton armUpButton = new JoystickButton(m_driverController1, 4);
      JoystickButton armDownButton = new JoystickButton(m_driverController1, 1);
      JoystickButton inTakeButton = new JoystickButton(m_driverController1, 2);
       JoystickButton climbButton = new JoystickButton(m_driverController1, 5);
      JoystickButton climbButton2 = new JoystickButton(m_driverController1, 6);
      climbButton2.whileTrue(climb_p);
      climbButton.whileTrue(climb_r);
      climbButton2.whileFalse(climb_f);
      climbButton.whileFalse(climb_f);
      armUpButton.toggleOnTrue(armUp);
      armDownButton.toggleOnTrue(armDown);
      inTakeButton.whileTrue(inTake);
      shootButton.whileTrue(shoot);
      inTakeButton.whileFalse(stopIntake);
      shootButton.whileFalse(stopIntake);
      armUpButton.toggleOnFalse(armStop);
      armDownButton.toggleOnFalse(armStop);

    }
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return 
     */
    
    

    public Command getAutonomousCommand() {
      //  // Create config for trajectory
      //   TrajectoryConfig config = new TrajectoryConfig(
      //     AutoConstants.kMaxSpeedMetersPerSecond,
      //      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      //     // Add kinematics to ensure max speed is actually obeyed
      //      .setKinematics(DriveConstants.kDriveKinematics);
  
      // // An example trajectory to follow. All units in meters.
      // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      //     // Start at the origin facing the +X direction
      //     new Pose2d(0, 0, new Rotation2d(0)),
      //     // Pass through these two interior waypoints, making an 's' curve path
      //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      //     // End 3 meters straight ahead of where we started, facing forward
      //     new Pose2d(3, 0, new Rotation2d(0)),
      //     config);
  
      // var thetaController = new ProfiledPIDController(
      //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      // thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      //     exampleTrajectory,
      //     m_robotDrive::getPose, // Functional interface to feed supplier
      //     DriveConstants.kDriveKinematics,
  
      //     // Position controllers
      //     new PIDController(AutoConstants.kPXController, 0, 0),
      //     new PIDController(AutoConstants.kPYController, 0, 0),
      //     thetaController,
      //     m_robotDrive::setModuleStates,
      //     m_robotDrive);
  
      // // Reset odometry to the starting pose of the trajectory.
      // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
   
      // // Run path following command, then stop at the end.
      // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));

      // return autoChooser.get();
      return autoChooser.getSelected();
    }
  // *  return null;}
    public void logTheBits() {
      // DriverStation.reportError(arm.getMeasurement()  +"", false);
    }
  
    public static void Test_controller() {
      // double potato = m_driverController.getLeftX();
      // int i = 1;
      // while (i < 5) {
      //   System.out.println(potato);
      //   i++;
      // }
  }
}
