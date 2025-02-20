package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class Elevator {
    private boolean isUp = false;
    private SparkMax elevatorMotor;
    private static final double ELEVATOR_UP_SPEED = 1;
    private static final double ELEVATOR_DOWN_SPEED = 1;
    private static final double HOLD_POWER = 1;
    private static final double MOVE_TIME = 1;
    private final Timer timer = new Timer();


    public Elevator(){
        elevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        //config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void toggleElevator(){ // move up or down
        timer.reset();
        timer.start();
        if (isUp){
            elevatorMotor.set(ELEVATOR_DOWN_SPEED);        }
        else{
            elevatorMotor.set(ELEVATOR_UP_SPEED);        }
        }
    
    // this goes somewhere in teleop or wherever the robot actually works i think
    public void update() {
        if (timer.get() >= MOVE_TIME) {
            if (isUp) {
                elevatorMotor.set(HOLD_POWER); // Apply hold power to keep the elevator up
            } else {
                elevatorMotor.set(0); // Stop the motor completely when down
            };
            isUp = !isUp;
            timer.stop();
        }
    }
    // at the end it should always go down so it matches isUp = false when the code starts running but idk where to put that

    
}