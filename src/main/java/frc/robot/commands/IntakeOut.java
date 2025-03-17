package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.InTakeOutPut;

  public class IntakeOut extends Command {
    /** Creates a new ClawGrab. */
    InTakeOutPut intakeOutPut;
    public IntakeOut(InTakeOutPut subsystem) {
      intakeOutPut = subsystem;
      addRequirements(subsystem);
      // Use addRequirements() here to declare subsystem dependencies.
    }
  
  
    
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      intakeOutPut.OutPut();
    }
    @Override
    public void end(boolean interrupted) {
    }
    @Override
  public boolean isFinished() {
      return false;
  }

}
