package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToTrackedTargetCommand extends Command {

    private DriveSubsystem m_drivetrainSubsystem;
    private VisionSubsystem m_visionSubsystem;

    private double desiredDistanceToTarget; // Desired distance to target in meters
    private int targetTagID; // Target ID for specific AprilTag
    private boolean usingArea; // Whether to use area or distance for control

    private double translationalError; // Error in distance from the target
    private double rotationValue; // Value to rotate the robot (turn to target)
    private double error; // Yaw error to correct for (from vision)

    public DriveToTrackedTargetCommand(double distanceToTarget) {
        m_drivetrainSubsystem = RobotContainer.m_robotDrive;
        m_visionSubsystem = RobotContainer.m_visionSubsystem;
        desiredDistanceToTarget = distanceToTarget;
        addRequirements(m_drivetrainSubsystem, m_visionSubsystem);
    }

    public DriveToTrackedTargetCommand(double distanceToTarget, int targetTagID) {
        this(distanceToTarget);
        this.targetTagID = targetTagID;
    }

    public DriveToTrackedTargetCommand(double targetArea, boolean usingArea) {
        this(targetArea);
        this.usingArea = usingArea;
    }

    public DriveToTrackedTargetCommand(double targetArea, int targetTagID, boolean usingArea) {
        this(targetArea, targetTagID);
        this.usingArea = usingArea;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if (!RobotContainer.isTrackingEnabled) {
            m_drivetrainSubsystem.drive(0, 0, 0, false, true); // Stop moving if tracking is disabled
            return;
        }
        
        if (m_visionSubsystem.getHasTarget()) {
            PhotonTrackedTarget trackedTarget;
            if(targetTagID == 0) {
                trackedTarget = m_visionSubsystem.getBestTarget();
            } else {
                trackedTarget = m_visionSubsystem.getTargetWithID(targetTagID);
            }

            if (trackedTarget != null) {
                // Calculate rotational error (yaw)
                double rotationalError = trackedTarget.getYaw();
                rotationValue = -rotationalError * Constants.TRACKED_TAG_ROATION_KP;

                // If the robot is aligned (yaw error is small), set rotation value to zero
                if (Math.abs(rotationalError) < 1) {
                    rotationValue = 0; // Stop rotating when aligned
                }

                // Calculate translational error (distance to target)
                double translationValue = 0;
                if (usingArea) {
                    translationalError = desiredDistanceToTarget - trackedTarget.getArea();
                    translationValue = translationalError * Constants.TRACKED_TAG_AREA_DRIVE_KP;
                } else {
                    translationalError = -desiredDistanceToTarget + PhotonUtils.calculateDistanceToTargetMeters(
                        Constants.CAMERA_HEIGHT_METERS,
                        Constants.TARGET_HEIGHT_METERS,
                        Constants.CAMERA_PITCH_RADIANS,
                        trackedTarget.getPitch()
                    );
                    translationValue = translationalError * Constants.TRACKED_TAG_DISTANCE_DRIVE_KP;
                }

                // Translate the values into forward, strafe, and rotation
                double forward = translationValue; // Forward translation (xSpeed)
                double strafe = 0; // No strafing, set to 0 as rotation handles turning (ySpeed)
                double rotation = rotationValue; // Rotation value (rot)

                // Drive using swerve
                boolean fieldRelative = false; // Set to true for field-relative control if needed
                boolean rateLimit = true; // Set to true for rate limiting

                m_drivetrainSubsystem.drive(forward, strafe, rotation, fieldRelative, rateLimit);

                // If translational error is small, stop moving
                if (Math.abs(translationalError) <= 0.2 && Math.abs(rotationalError) < 1) {
                    m_drivetrainSubsystem.drive(0, 0, 0, fieldRelative, rateLimit); // Stop moving when aligned
                }
            }
        } else {
            m_drivetrainSubsystem.drive(0, 0, 0, false, true); // No target, stop moving
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(translationalError) <= 0.2 && Math.abs(rotationValue) < 1; // Finished when aligned and close
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, false, true); // Stop when command ends or is interrupted
    }
}