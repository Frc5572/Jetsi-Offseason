package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.GenerateEmptyIO;

@GenerateEmptyIO({int.class, PhoenixOdometryThread.class})
public interface SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleInputs {

    }

    public void updateInputs(SwerveModuleInputs inputs);

}
