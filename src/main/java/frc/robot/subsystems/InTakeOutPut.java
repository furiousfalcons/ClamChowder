package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class InTakeOutPut {
        private NetworkTable tableInTakeOutPut = NetworkTableInstance.getDefault().getTable("InTakeOutPut");
    private SparkMax intakeOutPutMotor;
    private long time;
    private boolean isInTaking;
    private boolean isOutPutting;
    int proximity;

    public InTakeOutPut(){
        //work on the next line(WIP)
        intakeOutPutMotor = new SparkMax(Constants.INTAKEOUTPUT_MOTOR_ID, MotorType.kBrushless); 
        time = System.currentTimeMillis();
    }

    public void periodic() {
    }

    public void intake() {
        isInTaking = true;
        intakeOutPutMotor.set(Constants.intakeMotorSpeed);
        tableInTakeOutPut.getEntry("InTake").setBoolean(true);
    }

 public void OutPut() {
    if (isInTaking){
        isOutPutting = true;
        intakeOutPutMotor.set(-Constants.intakeMotorSpeed);
        tableInTakeOutPut.getEntry("Output").setBoolean(true);
    }
 }

 public void stop() {
    isOutPutting = false;
    isInTaking = false;
    intakeOutPutMotor.set(0.0);
    tableInTakeOutPut.getEntry("OutPut").setBoolean(false);
    tableInTakeOutPut.getEntry("InTake").setBoolean(false);
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