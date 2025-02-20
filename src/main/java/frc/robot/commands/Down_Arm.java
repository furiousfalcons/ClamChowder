package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Down_Arm extends Command {
  /** Creates a new ArmDown. */
  Arm arm;
  public Down_Arm(Arm subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.\
     arm = subsystem;
     addRequirements(subsystem);
     }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.armDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
}
