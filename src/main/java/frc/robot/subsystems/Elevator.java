package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;



public class Elevator extends SubsystemBase{
   private SparkMax elevatorMotor;
   private RelativeEncoder relativeEncoder;
    private static final double ELEVATOR_UP_SPEED = -1;
    private static final double ELEVATOR_DOWN_SPEED = 1;
    private static final double HOLD_POWER = 0;
    private static final double MOVE_TIME = 1;
    private final PIDController pidController;
    private final Timer timer = new Timer();
    public int i = 0;
    public boolean isUp;
    public boolean isDown;
    public final double minPosition = 0.1;
    public final double maxPosition = 5.5; //Diff: 6.43

    public Elevator(){



        elevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushed);
        relativeEncoder = elevatorMotor.getEncoder();
        pidController = new PIDController(0.11, 0.0, 0.0);
        pidController.setTolerance(0.3);


        SparkMaxConfig config = new SparkMaxConfig();
        //config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stopToggle(){
        elevatorMotor.set(HOLD_POWER);
    }

    // public void toggleElevator(double leftTrigger, double rightTrigger){
    //     double posit_elev = Math.abs(relativeEncoder.getPosition());
    //     double inputLeft = leftTrigger*ELEVATOR_UP_SPEED;
    //     double inputRight = rightTrigger*ELEVATOR_DOWN_SPEED;
    //     double minPosition = 0.1;
    //     double maxPosition = 5.5; //Diff: 6.43
    //     System.out.println(Math.abs(relativeEncoder.getPosition()) + "  " + (pidController.getSetpoint() >= relativeEncoder.getPosition()) + " " + pidController.getSetpoint());

    //     // If none of the buttons are pressed, the elevator doesn't move
    //     if((inputLeft != 0) && (inputRight != 0)){
    //         elevatorMotor.set(HOLD_POWER);
    //     } 
    //     // If the left button is pressed and the right button is not: the elevator moves down.
    //     else if((inputLeft != 0) && (inputRight == 0)){
    //         System.out.println("down");
    //         pidController.setSetpoint(minPosition);
    //         // Allow elevator to move down for as long as it is not on the bottom limit
    //         if(pidController.getSetpoint() >= relativeEncoder.getPosition()){
    //             elevatorMotor.set(inputLeft);
    //         // Keep elevator from going past limit
    //         } else if(pidController.atSetpoint() == true){
    //             elevatorMotor.set(HOLD_POWER);
    //         } 
    //     } else if((inputRight != 0) && (inputLeft == 0)){
    //         System.out.println("up");
    //         pidController.setSetpoint(maxPosition);
    //         // Allow elevator to move up for as long as it is not on the top limit
    //         if(pidController.getSetpoint() <= relativeEncoder.getPosition()){
    //             elevatorMotor.set(inputRight);
    //         } else if(pidController.atSetpoint() == true){
    //             elevatorMotor.set(HOLD_POWER);
    //         }
    //        } else {
    //         elevatorMotor.set(HOLD_POWER);
    //     }
    // }

    public void elevatorUp(){
        pidController.setSetpoint(maxPosition);

        if((pidController.getSetpoint() <= Math.abs(relativeEncoder.getPosition())) == false){
            elevatorMotor.set(ELEVATOR_DOWN_SPEED);
        } else if((pidController.getSetpoint() <= Math.abs(relativeEncoder.getPosition())) == true){
            elevatorMotor.set(HOLD_POWER);
        }
    }

    public void elevatorDown(){
        pidController.setSetpoint(minPosition);

        if((pidController.getSetpoint() >= Math.abs(relativeEncoder.getPosition())) == false){
            elevatorMotor.set(ELEVATOR_UP_SPEED);
        } else if((pidController.getSetpoint() <= Math.abs(relativeEncoder.getPosition())) == true){
            elevatorMotor.set(HOLD_POWER);
        }
    }

    @Override
    public void periodic(){
        System.out.println(relativeEncoder.getPosition() + "  " + (pidController.getSetpoint() >= relativeEncoder.getPosition()));
    }
    
    public void update() {
        
        // double minPosition = 2.5;
        // double maxPosition = 8.993;

        // if (relativeEncoder.getPosition() <= minPosition) {
        //     elevatorMotor.set(HOLD_POWER); // Stop motor at lower limit
        // } else if (relativeEncoder.getPosition() >= maxPosition) {
        //     elevatorMotor.set(HOLD_POWER); // Stop motor at upper limit
        // }
    }
}
