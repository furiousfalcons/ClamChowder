package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class Climb extends SubsystemBase {  
    private SparkMax climbMotor;
    private static final double CLIMB_SPEED = -0.4;

    public Climb(){

        climbMotor = new SparkMax(10,MotorType.kBrushed);
        SparkMaxConfig config = new SparkMaxConfig();

        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);





    }

    public void climbGo(){
        climbMotor.set(CLIMB_SPEED);
    }

    public void ClimbDown(){
        climbMotor.set(-CLIMB_SPEED);
    }

    public void climbStop(){
        climbMotor.set(0);
    }
}