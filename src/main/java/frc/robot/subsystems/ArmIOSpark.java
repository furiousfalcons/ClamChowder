package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ArmIOSpark implements ElevatorIO {
   private SparkMax armMotorL;
    private AbsoluteEncoder armEncoder;

  public ArmIOSpark() {
    armMotorL = new SparkMax(4, MotorType.kBrushed);
        armEncoder = armMotorL.getAbsoluteEncoder();
    SparkMaxConfig config = new SparkMaxConfig();
  }


  
}