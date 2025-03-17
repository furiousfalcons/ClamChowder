package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InTakeOutPut;

public class MasterCommands {
//     public static Command setElevatorPosition(
//         Elevator elevator, Arm arms, double elevatorPosition, double armPosition) {
//  return sequence(
//     runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
//     waitSeconds(0.25),
//     runOnce(() -> elevator.setPosition(elevatorPosition), elevator),
//     waitSeconds(0.25),
//     runOnce(()-> arms.setPosition(armPosition), arms));
//     }
    
//     public static Command setArmPosition(Arm arms, Elevator elevator) {
//         return sequence(
//         runOnce(() -> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
//         waitSeconds(.25),
//         runOnce(() -> elevator.setPosition(Constants.ElevatorConstants.ELEVATOR_REST), elevator),
//         waitSeconds(0.25),
//         runOnce(() -> arms.setPosition(Constants.ArmConstants.ARM_CORAL_LOADING_POSITION), arms));
        
//     }

    public static Command stopIntake(InTakeOutPut intake) {
        return runOnce(() -> intake.stop(), intake);
    }

    public static Command runIntake(InTakeOutPut intake) {
        return runOnce(() -> intake.intake(), intake);
    }

    public static Command runOutPut(InTakeOutPut outPut) {
        return runOnce(() -> outPut.OutPut(), outPut);
    }

    public static Command stopElevator(Elevator elevator) {
        return runOnce(() -> {elevator.stopToggle();}, elevator);
    }
}
