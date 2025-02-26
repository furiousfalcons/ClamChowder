package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.InTakeOutPut;

public class IntakeOut extends Command {

        InTakeOutPut intake;
        /** Creates a new ArmUp. */
        public IntakeOut(InTakeOutPut subsystem) {
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
    intake.intake();
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