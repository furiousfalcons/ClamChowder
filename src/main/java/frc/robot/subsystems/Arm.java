package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Arm extends SubsystemBase{

    private SparkMax armMotorL;
    // private DutyCycleEncoder armEncoder;
    private AbsoluteEncoder armEncoder;
    private final PIDController pidController;

    private static final double offset = 0.0;
    private static final double ARM_SPEED = 0.5;
    private static final double MOVE_TIME = 1.5;
    private static final double BOTTOM_LIMIT = 0.0 - offset;
    private static final double TOP_LIMIT = 0.615 - offset;
    private static final boolean test_boolean = true;

public Arm(){

    
    pidController = new PIDController(0.11, 0.0, 0.0);
    armMotorL = new SparkMax(4, MotorType.kBrushed);
    armEncoder = armMotorL.getAbsoluteEncoder();
    pidController.setTolerance(0.04);


    // SparkMaxConfig config = new SparkMaxConfig();        
    // config.inverted(true);
    // config.idleMode(IdleMode.kBrake);
    // config.closedLoop.pid(0.011, 0.0, 0.0);


    // pidController.setSetpoint(20);0.072 High
    // armMotorL.setCANTimeout(250); 0.429 low

}

// public void armUp() { 
//     armMotorL.set(ARM_SPEED);
// }

// public void armDown() {  
//     armMotorL.set(-0.1);
// }

public void armUp(){
    pidController.setSetpoint(TOP_LIMIT);
    double num = 1/(pidController.calculate(armEncoder.getPosition()));
    // System.out.println("one :" + num);
    // System.out.pri ntln("two:" + armEncoder.getPosition());
    // System.out.println("three: "+ pidController.atSetpoint());
    if (pidController.atSetpoint() == false){
        armMotorL.set(ARM_SPEED);
    }
    else if(pidController.atSetpoint() == true){
        armMotorL.set(0.18);
    }
}
public void armDown(){
    pidController.setSetpoint(BOTTOM_LIMIT);

    // armMotorL.set(MathUtil.clamp((pidController.calculate(armEncoder.getPosition(), BOTTOM_LIMIT)),-0.1, ARM_SPEED));
    if (pidController.atSetpoint() == false){
        armMotorL.set(-0.1);
    } else if(pidController.atSetpoint() == true){
        armMotorL.set(0.18);
    }
}

    
public double getMeasurement() {
    return  armEncoder.getPosition();
}



 @Override
 public void periodic(){
    // double num = armEncoder.getPositionConversionFactor();
    double num = armEncoder.getPosition();
    // System.out.println("Offset " + num);
 }
    

public void armStop(){
    armMotorL.set(0.18);
}

}