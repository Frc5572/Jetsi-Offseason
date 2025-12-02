package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public static class GyroInputs {

    }

    public void updateInputs(GyroInputs inputs);

    public static class Empty implements GyroIO {
        public Empty() {}

        @Override
        public void updateInputs(GyroInputs inputs) {
            // Intentionally do nothing
        }
    }

}
