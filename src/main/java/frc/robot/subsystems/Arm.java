package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class Arm extends SubsystemBase{

    private SparkMax armMotorL;
    private DutyCycleEncoder armEncoder;
    private final PIDController pidController;
    public int i = 0;


    public Arm(){

        

        armMotorL = new SparkMax(Constants.armMotorL, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(Constants.armEncoder);

        pidController = new PIDController(0.011, 0.000, 0.000);
        pidController.setTolerance(2.0);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);
        config.closedLoop.pid(0.011, 0.0, 0.0);

        armMotorL.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
        armMotorL.setCANTimeout(250);

        // getController().setTolerance(10);

        armMotorL.setCANTimeout(250);
    }

    
    public void armDown(int num) { 
        if (num%2 == 0){
            //double distance = pidController.getSetpoint() + 2;
            pidController.setSetpoint(270);
        }
        if (num%2 == 1){
            pidController.setSetpoint(pidController.getSetpoint());
        }

    }

    public void armUp(int num) { 
        if (num%2 == 0){
            //double distance = pidController.getSetpoint() - 2;
            pidController.setSetpoint( 135) ;
        }
        if (num%2 == 1){
            pidController.setSetpoint(pidController.getSetpoint());
        }

    }

    
    public double getMeasurement() {
        return  armEncoder.get(); //rotations not position unfortnately :()
    }

    
    protected void useOutput(double output, double setpoint) {
        System.out.println(output);
     }

    
}