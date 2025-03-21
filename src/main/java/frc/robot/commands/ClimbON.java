 package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;

public class ClimbON extends Command{
    Elevator arm;
  public ClimbON(Elevator subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.\
     arm = subsystem;
     addRequirements(subsystem);
     }

     public int i = 2;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // arm.armDown();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // arm.armDown(i);
    // i++;
    arm.elevatorUp();

    // arm.moveArmDown();
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





