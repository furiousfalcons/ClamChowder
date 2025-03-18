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
    // private Encoder armEncoder;
private AbsoluteEncoder armEncoder;
    // private final PIDController pidController;

    private static final double ARM_SPEED = 0.5;
    private static final double MOVE_TIME = 1.5;


    public Arm(){

        
        armMotorL = new SparkMax(4, MotorType.kBrushed);
        // armEncoder = armMotorL.getAbsoluteEncoder();


        // pidController = new PIDController(0.011, 0.000, 0.000);
        // pidController.setTolerance(2.0);
        // pidController.setSetpoint(getMeasurement());

        // SparkMaxConfig config = new SparkMaxConfig();
        // config.inverted(true);
        // config.idleMode(IdleMode.kBrake);
        // config.closedLoop.pid(0.011, 0.0, 0.0);


        // pidController.setSetpoint(20);
        // armMotorL.setCANTimeout(250);

        // MathUtil.clamp(pidController.calculate(armEncoder.get(), setpoint), -0.5, 0.5);

    
    }

    public void armUp() { 
        armMotorL.set(ARM_SPEED);
    }

    public void armDown() {
        armMotorL.set(-0.1);

    }


    
    // public double getMeasurement() {
    //     return  armEncoder.get();
    //      //rotations not position unfortnately :()
    // }

    
    // protected void useOutput(double output, double setpoint) {
    //     System.out.println(output);
    //  }

    //  @Override
    //  public void periodic(){
    //     // double num = armEncoder.getPositionConversionFactor();
    //     double num = armEncoder.getPosition();
    //     System.out.println(num);
    
    //  }
        
    // public void armDown(int num) { 
    //     if (num%2 == 0){
    //         //double distance = pidController.getSetpoint() + 2;
    //         pidController.setSetpoint(270);
    //     }
    //     if (num%2 == 1){
    //         pidController.setSetpoint(pidController.getSetpoint());
    //     }

    // }

    // public void armUp(int num) { 
    //     if (num%2 == 0){
    //         //double distance = pidController.getSetpoint() - 2;
    //         pidController.setSetpoint( 135) ;
    //     }
    //     if (num%2 == 1){
    //         pidController.setSetpoint(pidController.getSetpoint());
    //     }

    // }

    public void armStop(){
        armMotorL.set(0.18);
    }



    
}
