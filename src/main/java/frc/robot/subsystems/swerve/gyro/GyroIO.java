package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public static class GyroInputs {
        public double yawRads;
    }

    public void updateInputs(GyroInputs inputs);

}
