package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class Up_Arm extends Command {

        Arm arm;
        /** Creates a new ArmUp. */
        public Up_Arm(Arm subsystem) {
          // Use addRequirements() here to declare subsystem dependencies.
          arm = subsystem;
          addRequirements(subsystem);
        }

        public int i = 2;

     // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // arm.armUp(i);
    // i++;
    // arm.armUp();
    arm.moveArmUp();
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