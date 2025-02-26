// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import frc.robot.Constants;



// public class Elevator extends SubsystemBase{
//    private SparkMax elevatorMotor;
//     private static final double ELEVATOR_UP_SPEED = 0.3;
//     private static final double ELEVATOR_DOWN_SPEED = -0.3;
//     private static final double HOLD_POWER = 0;
//     public int i = 0;


//     public Elevator(){
//         elevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushed);

//         SparkMaxConfig config = new SparkMaxConfig();
//         //config.inverted(true);
//         config.idleMode(IdleMode.kBrake);
//         config.smartCurrentLimit(40);

//         elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     public void toggleUp(int num){ // move up or
        
//             if(num%2 == 0){
//         elevatorMotor.set(ELEVATOR_UP_SPEED);
//             } if(num%2 == 1) {
//         elevatorMotor.set(HOLD_POWER);
//             }
//         }
//         public void toggleDown(int num){
//             if(num%2 == 0){
//                 elevatorMotor.set(ELEVATOR_DOWN_SPEED);
//             } if(num%2 == 1) {
//                 elevatorMotor.set(HOLD_POWER);
//                     }
//         }
    
//     // this goes somewhere in teleop or wherever the robot actually works i think
//     public void update() {

//     }
//     // at the end it should always go down so it matches isUp = false when the code starts running but idk where to put that

//     public void toggleElevator() {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'toggleElevator'");
//     }

    
// }