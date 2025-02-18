
package frc.Commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.InTakeOutPut;

public class InTake extends Command {
  /** Creates a new IntTake. */

InTakeOutPut intakeOutPut;

  public InTake(InTakeOutPut subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeOutPut = subsystem;
      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeOutPut.intake();
    SmartDashboard.putBoolean("Is Picking Up", intakeOutPut.isCurrentlyInTaking());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeOutPut.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}