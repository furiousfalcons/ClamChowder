package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InTakeOutPut extends SubsystemBase{
    private SparkMax intakeOutPutMotor; 
    private SparkMax intakeOutPutMotor2;
    private boolean isInTaking;
    private boolean isOutPutting;
    private I2C.Port i2cPort;
    int proximity;

    public InTakeOutPut() {
        i2cPort = I2C.Port.kOnboard;
        // colorSensor = new ColorSensorV3(i2cPort);
        //work on the next line(WIP)
        intakeOutPutMotor = new SparkMax(Constants.INTAKEOUTPUT_MOTOR_ID_1, MotorType.kBrushless); 
        intakeOutPutMotor2 = new SparkMax(14, MotorType.kBrushless);
    }

    public void periodic() {
        // proximity = colorSensor.getProximity();
    }

    public void intake() {
        intakeOutPutMotor.set(-Constants.inTakeMotorSpeed);
        intakeOutPutMotor2.set(Constants.inTakeMotorSpeed);
    }

 public void OutPut() {
 
        intakeOutPutMotor.set(Constants.inTakeMotorSpeed);
        intakeOutPutMotor2.set(-Constants.inTakeMotorSpeed);
 }

 public void stop() {
    isOutPutting = false;
    isInTaking = false;
    intakeOutPutMotor.set(0.0);
    intakeOutPutMotor2.set(0.0);
 }

 public boolean isCurrentlyInTaking()
 {
    return isInTaking;
 }
 public boolean isCurrentlyOutPuting()
 {
    return isOutPutting;
 }
}