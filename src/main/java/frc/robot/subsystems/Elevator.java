package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
   private SparkMax elevatorMotor;
       private final RelativeEncoder elevatorEncoder;
    private static final double ELEVATOR_UP_SPEED = -1;
    private static final double ELEVATOR_DOWN_SPEED = 1;
    private static final double HOLD_POWER = 0;
    private static final double MOVE_TIME = 1;
    private final Timer timer = new Timer();
    public int i = 0;
    public boolean isUp;
    private double targetPosition;
    private double targetSpeed = 0;
    private final PIDController pidController;
    private ArmFeedforward feedForward;

    public Elevator(){
        elevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushed);
        elevatorEncoder = elevatorMotor.getEncoder();

        pidController = new PIDController(2.5, 0, 0.1);
        pidController.setTolerance(.25);
        feedForward =
            new ArmFeedforward(0.11237, 0.76416, 0.56387, 0.041488);
    
            SparkMaxConfig config = new SparkMaxConfig();
            //config.inverted(true);
            config.idleMode(IdleMode.kBrake);
            config.smartCurrentLimit(40);
    
            elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

    public void periodic() {
        Logger.recordOutput("Elevator Position", getPosition());

        double pidMotorSpeed = 
            pidController.calculate(getPosition(), targetPosition)
                + feedForward.calculate(targetPosition, 0);
            Logger.recordOutput("PID Speed", pidMotorSpeed);
            Logger.recordOutput("Elevator Speed" , targetSpeed);

            setMotor(
                MathUtil.clamp(
                (pidMotorSpeed),
                 -ElevatorConstants.MAX_ELEVATOR_VOLTAGE,
                  ElevatorConstants.MAX_ELEVATOR_VOLTAGE));
            
    }
    
        // public void toggleUp(int num){ // move up or
            
        //         if(num%2 == 0){
        //     elevatorMotor.set(ELEVATOR_UP_SPEED);
        //         } if(num%2 == 1) {
        //     elevatorMotor.set(HOLD_POWER);
        //         }
        //     }
        //     public void toggleDown(int num){
        //         if(num%2 == 0){
        //             elevatorMotor.set(ELEVATOR_DOWN_SPEED);
        //         } if(num%2 == 1) {
        //             elevatorMotor.set(HOLD_POWER);
        //                 }
        //     }
    
        public void stopToggle(){
        elevatorMotor.set(HOLD_POWER);
    }

    public void toggleElevator(double leftTrigger, double rightTrigger){
        double inputLeft = leftTrigger*ELEVATOR_UP_SPEED;
        double inputRight = rightTrigger*ELEVATOR_DOWN_SPEED;
        if((inputLeft != 0) && (inputRight != 0)){
            elevatorMotor.set(HOLD_POWER);
        } else if((inputLeft != 0) && (inputRight == 0)){
            elevatorMotor.set(inputLeft);
        } else if((inputRight != 0) && (inputLeft == 0)){
            elevatorMotor.set(inputRight);
        } else {
            elevatorMotor.set(HOLD_POWER);
        }

    }

    public void elevatorDown(){
        elevatorMotor.set(ELEVATOR_DOWN_SPEED);
    }
    public void elevatorUp(){
        elevatorMotor.set(ELEVATOR_UP_SPEED);
    }

    public void setPosition(double position) {
        Logger.recordOutput("ElevatorTargetPosition", position);
        targetPosition = position;
    }

    public double getPosition() {
        return elevatorEncoder.getPosition();
    }


public void setSpeed(double speed) {
    targetSpeed = speed;
}

public void setElevatorSetpoint(double offset) {
    setPosition(getPosition() + offset);
}

public void setMotor(double voltage) {
    setElevatorVoltage(voltage);
    }
    
    private void setElevatorVoltage(double voltage) {}
    
}


  



// public void resetPosition() {
//     resetPosition();
// }

// public void setSpeed(double speed) {
//     targetSpeed = speed;
// }

// public void setElevatorSetpoint(double offset) {
//     setPosition(getPosition() + offset);
// }

// public void setMotor(double voltage) {
//     setElevatorVoltage(voltage);
//     }
    
//     private void setElevatorVoltage(double voltage) {}





    
    // this goes somewhere in teleop or wherever the robot actually works i think
    // public void update() {
    //     if (timer.get() >= MOVE_TIME) {
    //         if (isUp) {
    //             elevatorMotor.set(HOLD_POWER); // Apply hold power to keep the elevator up
    //         } else {
    //             elevatorMotor.set(0); // Stop the motor completely when down
    //         };
    //         isUp = !isUp;
    //         timer.stop();
    //     }
    // }
    // at the end it should always go down so it matches isUp = false when the code starts running but idk where to put that