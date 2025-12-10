package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.GenerateEmptyIO;

@GenerateEmptyIO({PhoenixOdometryThread.class})
public interface GyroIO {

    @AutoLog
    public static class GyroInputs {

    }

    public void updateInputs(GyroInputs inputs);

}
