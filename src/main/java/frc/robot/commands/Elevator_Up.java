package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Elevator_Up extends Command {

    Elevator lift;
    /** Creates a new ArmUp. */
    public Elevator_Up(Elevator subsystem) {
      // Use addRequirements() here to declare subsystem dependencies.
      lift = subsystem;
      addRequirements(subsystem);
    }

 // Called when the command is initially scheduled.
@Override
public void initialize() {

}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    lift.toggleElevator();
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