package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface LimeLightIO {

    @AutoLog
    public class LimeLightInputs {
        Pose2d[] cameraPose;
        double[] latestCapture = {0.0};
        int[] tagCount;
        double[] timestamp;
    }

    public default void updateInputs(LimeLightInputs inputs) {}

    public default void setRobotOrentation(Pose2d pose) {}
}
