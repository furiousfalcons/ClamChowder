package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;

public class ClimbON extends Command{
    Climb arm;
  public ClimbON(Climb subsystem) {
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
    arm.climbGo();

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