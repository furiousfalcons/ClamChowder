package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
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

    public static Command setElevatorPositionL1C(Arm arms, Elevator elevator, double l1Coral, double armRestPosition) {
             return sequence(
                runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
                waitSeconds(0.25),
                runOnce(() -> elevator.setPosition(Constants.ElevatorConstants.L1_CORAL), elevator),
                waitSeconds(0.25),
                runOnce(()-> arms.setPosition(Constants.ArmConstants.L1_CORAL_ARM), arms));  
              }

    public static Command setElevatorPositionL2C(Arm arms, Elevator elevator, double l2Coral, double armCoralPosition) {
        return sequence(
            runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
            waitSeconds(0.25),
            runOnce(() -> elevator.setPosition(Constants.ElevatorConstants.L2_CORAL), elevator),
            waitSeconds(0.25),
            runOnce(()-> arms.setPosition(Constants.ArmConstants.L2_CORAL_ARM), arms));  
    }

    public static Command setElevatorPositionL3C(Arm arms, Elevator elevator, double l3Coral, double armCoralPosition) {
        return sequence(
            runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
            waitSeconds(0.25),
            runOnce(() -> elevator.setPosition(Constants.ElevatorConstants.L3_CORAL), elevator),
            waitSeconds(0.25),
            runOnce(()-> arms.setPosition(Constants.ArmConstants.L3_CORAL_ARM), arms));  
    }

    public static Command setElevatorPositionL2A(Arm arms, Elevator elevator, double l2Algea, double armAlgeaPosition) {
        return sequence(
            runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
            waitSeconds(0.25),
            runOnce(() -> elevator.setPosition(Constants.ElevatorConstants.L2_ALGEA), elevator),
            waitSeconds(0.25),
            runOnce(()-> arms.setPosition(Constants.ArmConstants.L2_ALGEA_ARM), arms)); 
    }

    public static Command setElevatorPositionL3A(Arm arms, Elevator elevator, double l3Algea, double armAlgeaPosition) {
        return sequence(
            runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
            waitSeconds(0.25),
            runOnce(() -> elevator.setPosition(Constants.ElevatorConstants.L3_ALGEA), elevator),
            waitSeconds(0.25),
            runOnce(()-> arms.setPosition(Constants.ArmConstants.L3_ALGEA_ARM), arms)); 
    }

    // public static Command setElevatorPositionLAA(Arm arms, Elevator elevator, double ampAlgea, double armRestPosition) {
    //     return sequence(
    //         runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
    //         waitSeconds(0.25),
    //         runOnce(() -> elevator.setPosition(Constants.ElevatorConstants.AMP_ALGEA), elevator),
    //         waitSeconds(0.25),
    //         runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms)); 
    // }

    public static Command setElevatorPositionLI(Arm arms, Elevator elevator, double elevatorIntake,
            double armCoralLoadingPosition) {
        return sequence(
            runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_REST_POSITION), arms),
            waitSeconds(0.25),
            runOnce(()-> elevator.setPosition(Constants.ElevatorConstants.ELEVATOR_INTAKE), elevator),
            waitSeconds(0.25),
            runOnce(()-> arms.setPosition(Constants.ArmConstants.ARM_CORAL_LOADING_POSITION), arms));
    
    }
}
