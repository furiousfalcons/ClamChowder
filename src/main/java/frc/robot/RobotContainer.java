// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

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
import frc.robot.commands.DriveToTrackedTargetCommand;
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
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.InTakeOutPut;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final InTakeOutPut intake;
  private final Arm arm;
  private final Elevator elevator;
  // The robot's subsystems
    public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
    public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public static boolean isTrackingEnabled = false; // toggle searching for april tags
  // private InTakeOutPut intakeShooter = new InTakeOutPut();
  // private Elevator climb = new Elevator();
  // private Arm arm = new Arm();
   private Climb goClimb = new Climb();
  // private final IntakeIn inTake = new IntakeIn(intakeShooter);
  // private final IntakeOut shoot = new IntakeOut(intakeShooter);
  private final climbDown climb_p = new climbDown(goClimb);
  private final ClimbON climb_r = new ClimbON(goClimb);
  private final ClimbStop climb_f = new ClimbStop(goClimb);
  // private final Stop_Intake stopIntake = new Stop_Intake(intakeShooter);
  //  private final Up_Arm armUp = new Up_Arm(arm);
  // private final Down_Arm armDown = new Down_Arm(arm);
  // private final Stop_Arm armStop = new Stop_Arm(arm);
   private final DriveToTrackedTargetCommand april = new DriveToTrackedTargetCommand(2, 3); // vision command

 
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
arm = new Arm();

NamedCommands.registerCommand("Stop Intake", MasterCommands.stopIntake(intake));
NamedCommands.registerCommand("Run Intake", MasterCommands.runIntake(intake));
NamedCommands.registerCommand("Run Output", MasterCommands.runOutPut(intake));

// NamedCommands.registerCommand("Set Arm Position", MasterCommands.setArmPosition(arm, elevator));

NamedCommands.registerCommand(
            "Set Elevator L1 C",
            MasterCommands.setElevatorPositionL1C(
                arm,
                elevator,
                ElevatorConstants.L1_CORAL,
                ArmConstants.ARM_REST_POSITION));

                NamedCommands.registerCommand(
            "Set Elevator L2 C",
            MasterCommands.setElevatorPositionL2C(
                arm,
                elevator,
                ElevatorConstants.L2_CORAL,
                ArmConstants.ARM_CORAL_POSITION));

                 NamedCommands.registerCommand(
                  "Set Elevator L3 C",
                  MasterCommands.setElevatorPositionL3C(
                      arm,
                      elevator,
                      ElevatorConstants.L3_CORAL,
                      ArmConstants.ARM_CORAL_POSITION));

                      NamedCommands.registerCommand(
                  "Set Elevator L2 A",
                  MasterCommands.setElevatorPositionL2A(
                      arm,
                      elevator,
                      ElevatorConstants.L2_ALGEA,
                      ArmConstants.ARM_ALGEA_POSITION));

                      NamedCommands.registerCommand(
                  "Set Elevator L3 A",
                  MasterCommands.setElevatorPositionL3A(
                      arm,
                      elevator,
                      ElevatorConstants.L3_ALGEA,
                      ArmConstants.ARM_ALGEA_POSITION));

                      NamedCommands.registerCommand(
                  "Set Elevator Amp A",
                  MasterCommands.setElevatorPositionLAA(
                      arm,
                      elevator,
                      ElevatorConstants.AMP_ALGEA,
                      ArmConstants.ARM_REST_POSITION));
                        NamedCommands.registerCommand(
                     "Set Elevator Amp A",
                     MasterCommands.setElevatorPositionLI(
                        arm,
                        elevator,
                        ElevatorConstants.ELEVATOR_INTAKE,
                        ArmConstants.ARM_CORAL_LOADING_POSITION));

                        
                      

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

        

      // elevator.setDefaultCommand(
      //   new RunCommand(
      //     () -> elevator.toggleElevator(
      //       -MathUtil.applyDeadband(m_driverController2.getLeftTriggerAxis(), 0.05),
      //       -MathUtil.applyDeadband(m_driverController2.getRightTriggerAxis(), 0.05)
      //     ), 
      //   elevator));
           autoChooser = AutoBuilder.buildAutoChooser();
           SmartDashboard.putData("Auto Chooser", autoChooser);

 
// Real robot, instantiate hardware IO implementations
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

      JoystickButton LAAButton = new JoystickButton(m_driverController2, 1);
      JoystickButton inTakeButton = new JoystickButton(m_driverController2, 2);
      JoystickButton shootButton = new JoystickButton(m_driverController2, 3);
      JoystickButton L3AButton = new JoystickButton(m_driverController2, 4);
      JoystickButton L2AButton = new JoystickButton(m_driverController1, 5);
      JoystickButton L1CButton = new JoystickButton(m_driverController1, 6);
      JoystickButton L3CButton = new JoystickButton(m_driverController1, 7);
      JoystickButton L2CButton = new JoystickButton(m_driverController1, 8);
      JoystickButton LIButton = new JoystickButton(m_driverController1, 9);
      JoystickButton toggleTrackingButton = new JoystickButton(m_driverController1, 1); // does this go here or with the other controller1 stuff

       JoystickButton climbButton = new JoystickButton(m_driverController2, 11);
      JoystickButton climbButton2 = new JoystickButton(m_driverController2, 12);
      climbButton2.whileTrue(climb_p);
      climbButton.whileTrue(climb_r);
      climbButton2.whileFalse(climb_f);
      climbButton.whileFalse(climb_f);

       toggleTrackingButton.onTrue(new InstantCommand(() -> {  // toggle to turn on april tag detection
            isTrackingEnabled = !isTrackingEnabled; // Toggle tracking
            System.out.println("Tracking enabled: " + isTrackingEnabled);

            if (isTrackingEnabled) {    
                april.schedule(); // Start command
            } else {
                april.cancel(); // Stop command
            }
        }));



      L3AButton
      .onTrue(MasterCommands.setElevatorPositionL3A(
        arm,
        elevator,
        ElevatorConstants.L3_ALGEA,
        ArmConstants.ARM_ALGEA_POSITION));
      LAAButton
      .onTrue(MasterCommands.setElevatorPositionLAA(
        arm,
        elevator,
        ElevatorConstants.AMP_ALGEA,
        ArmConstants.ARM_REST_POSITION));

      L2AButton
      .onTrue(MasterCommands.setElevatorPositionLAA(
          arm,
          elevator,
          ElevatorConstants.AMP_ALGEA,
          ArmConstants.ARM_REST_POSITION));

      L3CButton
      .onTrue(MasterCommands.setElevatorPositionL3C(
        arm,
        elevator,
        ElevatorConstants.L3_CORAL,
        ArmConstants.ARM_CORAL_POSITION));

      L2CButton
      .onTrue(MasterCommands.setElevatorPositionL3C(
        arm,
        elevator,
        ElevatorConstants.L3_CORAL,
        ArmConstants.ARM_CORAL_POSITION));
      
      L1CButton
      .onTrue(MasterCommands.setElevatorPositionL3C(
        arm,
        elevator,
        ElevatorConstants.L1_CORAL,
        ArmConstants.ARM_CORAL_POSITION));

      LIButton
      .onTrue(MasterCommands.setElevatorPositionLI(
        arm,
        elevator,
        ElevatorConstants.ELEVATOR_INTAKE,
        ArmConstants.ARM_CORAL_LOADING_POSITION));

        

      inTakeButton
      .onTrue(MasterCommands.runIntake(intake))
      .onFalse(MasterCommands.stopIntake(intake));

      shootButton
      .onTrue(MasterCommands.runOutPut(intake))
      .onFalse(MasterCommands.stopIntake(intake));


    }
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return 
     */
    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
