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
    private NetworkTable tableInTakeOutPut = NetworkTableInstance.getDefault().getTable("InTakeOutPut");
    private SparkMax intakeOutPutMotor_1; 
    private SparkMax intakeOutPutMotor_2;
    private ColorSensorV3 colorSensor;
    private long time;
    private boolean isInTaking;
    private boolean isOutPutting;
    private I2C.Port i2cPort;
    int proximity;

    public InTakeOutPut() {
        i2cPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(i2cPort);
        //work on the next line(WIP)
        intakeOutPutMotor_1 = new SparkMax(Constants.inTakeMotor_2, MotorType.kBrushless);
        intakeOutPutMotor_2 = new SparkMax(Constants.inTakeMotor, MotorType.kBrushless); 
        time = System.currentTimeMillis();
    }

    public void periodic() {
        proximity = colorSensor.getProximity();
    }

    public void intake() {
        isInTaking = true;
        intakeOutPutMotor_1.set( Constants.inTakeMotorSpeed);
        intakeOutPutMotor_2.set( Constants.inTakeMotorSpeed);
        tableInTakeOutPut.getEntry("inTake").setBoolean(true);
    }

 public void OutPut() {
    if (isInTaking){
        isOutPutting = true;
        intakeOutPutMotor_1.set(-Constants.inTakeMotorSpeed);
        intakeOutPutMotor_2.set(-Constants.inTakeMotorSpeed);
        tableInTakeOutPut.getEntry("outPut").setBoolean(true);
    }
 }

 public void stop() {
    isOutPutting = false;
    isInTaking = false;
    intakeOutPutMotor_1.set(0.0);
    intakeOutPutMotor_2.set(0.0);
    tableInTakeOutPut.getEntry("OutPut").setBoolean(false);
    tableInTakeOutPut.getEntry("InTake").setBoolean(false);
 }

 public boolean isCurrentlyInTaking()
 {
    return isInTaking;
 }
 public boolean isCurrentlyOutPutting()
 {
    return isOutPutting;
 }
}