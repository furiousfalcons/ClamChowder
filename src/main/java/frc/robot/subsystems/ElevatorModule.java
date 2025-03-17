// package frc.robot.subsystems;

// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import edu.wpi.first.math.util.Units;


// import frc.robot.Constants;

// public class ElevatorModule {
//     private final SparkMax elevatorMotor;

//     SparkMaxConfig elevatorconfig = new SparkMaxConfig();
//     ClosedLoopConfig elevatorClosedLoopConfig = new ClosedLoopConfig();

//     private final RelativeEncoder elevatorEncoder;

//     private final SparkClosedLoopController elevatorClosedLoopController;

//     public ElevatorModule() {
//         elevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushed);

//         elevatorEncoder = elevatorMotor.getEncoder();

//         elevatorClosedLoopController = elevatorMotor.getClosedLoopController();

//     elevatorClosedLoopConfig.pidf(getPosition(), getPosition(), getPosition(), getPosition());
//     elevatorClosedLoopConfig.outputRange(0, 0);

//     elevatorconfig.idleMode(null);
//     elevatorconfig.smartCurrentLimit(0);

//     elevatorMotor.configure(elevatorconfig, null, null);
//     elevatorMotor.configure(elevatorconfig, null, null);

//     elevatorEncoder.setPosition(0);
// }

// public ElevatorModule getState() {
//     return new ElevatorState(elevatorEncoder.getVelocity());
//  }

//  public ElevatorPosition getPosition(){
//     return new ElevatorPosition(elevatorEncoder.getPosition());
//  }

//  public void resetEncoders() {
//     elevatorEncoder.setPosition(0);
//  }
//  }