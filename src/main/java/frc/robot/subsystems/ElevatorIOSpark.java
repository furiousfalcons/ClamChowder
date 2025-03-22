package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ElevatorIOSpark implements ElevatorIO {
   private SparkMax elevatorMotor;
       private final RelativeEncoder elevatorEncoder;

  public ElevatorIOSpark() {
    elevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushed);
        elevatorEncoder = elevatorMotor.getEncoder();
    SparkMaxConfig config = new SparkMaxConfig();
  }


  
}