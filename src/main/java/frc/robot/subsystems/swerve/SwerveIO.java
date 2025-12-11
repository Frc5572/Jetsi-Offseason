package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.util.GenerateEmptyIO;

@GenerateEmptyIO(PhoenixOdometryThread.class)
public interface SwerveIO {

    @AutoLog
    public static class SwerveInputs {
        public double[] timestamps = new double[0];
    }

    public void updateInputs(SwerveInputs inputs);

}
