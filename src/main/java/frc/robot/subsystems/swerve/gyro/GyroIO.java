package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;

public interface GyroIO {

    @AutoLog
    public static class GyroInputs {

    }

    public void updateInputs(GyroInputs inputs);

    public static class Empty implements GyroIO {
        public Empty(PhoenixOdometryThread odometryThread) {}

        @Override
        public void updateInputs(GyroInputs inputs) {
            // Intentionally do nothing
        }
    }

}
