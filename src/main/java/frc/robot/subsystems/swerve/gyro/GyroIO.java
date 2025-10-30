package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public interface GyroIO {

    @AutoLog
    public static class GyroInputs {
        public Rotation2d yaw;

        public Rotation2d[] odometryYaw =
            new Rotation2d[Constants.Swerve.MAX_ODOMETRY_SUBTICK_MEASUREMENTS];
    }

    public void updateInputs(GyroInputs inputs);

}
