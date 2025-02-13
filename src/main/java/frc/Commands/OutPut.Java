package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.InTakeShooter;


public class OutPut extends Command {
  /** Creates a new ClawGrab. */
  InTakeOutPut intakeOutPut;
  public OutPut(InTakeOutPut subsystem) {
    intakeOutPut = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeOutPut.OutPut();
    SmartDashboard.putBoolean("Is OutPutting", intakeOutPut.isCurrentlyOutPutting());
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