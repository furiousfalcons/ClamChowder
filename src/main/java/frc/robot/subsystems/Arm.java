package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class Arm{

    private SparkMax armMotorL;
    private SparkMax armMotorR;
    //private DutyCycleEncoder armEncoder;
    private final PIDController pidController;


    public Arm(){

        

        armMotorL = new SparkMax(Constants.armMotorL, MotorType.kBrushless);
        armMotorR = new SparkMax(Constants.armMotorR, MotorType.kBrushless);
        //armEncoder = new DutyCycleEncoder(Constants.armEncoder);
        //setSetpoint(getMeasurement());

        pidController = new PIDController(0.011, 0.000, 0.000);
        pidController.setTolerance(2.0);

                SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);
        config.closedLoop.pid(0.011, 0.0, 0.0);

        armMotorL.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armMotorL.setCANTimeout(250);
        armMotorR.setCANTimeout(250);

        //getController().setTolerance(10);

        armMotorL.setCANTimeout(250);
        armMotorR.setCANTimeout(250);
    }

    public void armUp() { 
        double distance = pidController.getSetpoint() - 2;
        pidController.setSetpoint(Math.max(distance, 135) );

    }

    public void armDown() { 
        double distance = pidController.getSetpoint() + 2;
        
        pidController.setSetpoint(Math.min(distance,270));

    }

    @Override
    public double getMeasurement() {
        //return  armEncoder.getAbsolutePosition(); //rotations not position unfortnately :()
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        System.out.println(output);
     }

    
}
