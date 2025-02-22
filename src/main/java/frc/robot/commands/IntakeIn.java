package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InTakeOutPut;

public class IntakeIn extends Command {

        InTakeOutPut intake;
        /** Creates a new ArmUp. */
        public IntakeIn(InTakeOutPut subsystem) {
          // Use addRequirements() here to declare subsystem dependencies.
          intake = subsystem;
          addRequirements(subsystem);
        }

     // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.OutPut();
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
