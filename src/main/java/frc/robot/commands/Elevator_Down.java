package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class Elevator_Down extends Command {

    Elevator lift;

    public Elevator_Down(Elevator subsystem) {
      // Use addRequirements() here to declare subsystem dependencies.
      lift = subsystem;
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
    // lift.toggleDown(i);
    // System.out.println(i);
    // i++;
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
