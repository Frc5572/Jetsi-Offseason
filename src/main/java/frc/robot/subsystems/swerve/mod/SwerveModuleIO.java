package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.util.swerve.PhoenixOdometryThread;

public interface SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleInputs {

    }

    public void updateInputs(SwerveModuleInputs inputs);

    public static class Empty implements SwerveModuleIO {
        public Empty(int index, PhoenixOdometryThread odometryThread) {}

        @Override
        public void updateInputs(SwerveModuleInputs inputs) {
            // Intentionally do nothing
        }
    }

}
