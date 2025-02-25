package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class Elevate {
    Elevator elevator;

    public Elevate(Elevator subsystem){
        elevator = subsystem;
        addRequirements(subsystem);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.toggleElevator();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end() {}

}
