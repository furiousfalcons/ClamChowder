package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
    public double armVelocityRadPerSec = 0.0;
    public double armAbsoluteEncoderPosition = 0.0;
    public double wristAppliedVolts = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setPosition(double position) {}
  
    default void setVoltage(double speed) {}
  
    default double getPosition() {
      return 0;
    }
}


