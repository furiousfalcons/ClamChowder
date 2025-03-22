package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double elevatorEncoderPosition = 0.0;
        double elevatorVelocityRadPerSec = 0d;
    }
    
    default void updateInputs(ElevatorIOInputs inputs) {}

    // Sets the power to the elevator motor
    default void setElevatorVoltage(double voltage) {}
  
    // Gets the current position of the elevator (in encoder units)
    default double getPosition() {
      return 0;
    }
  
    default void setPosition(double position) {}
  
    // Resets the encoder position to a specific value
    default void resetPosition() {}
  
    default void stop() {}

}
