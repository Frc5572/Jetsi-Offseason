package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

import java.util.Optional;

public interface LimeLightIO {

    @AutoLog
    public class LimeLightInputs {
        Pose2d cameraPoseOne;
        Pose2d cameraPoseTwo;
        double latestCaptureOne;
        double latestCaptureTwo;
        int tagCountOne;
        int tagCountTwo;
        double timestampOne;
        double timestampTwo;
    }

    public default void updateInputs(LimeLightInputs inputs) {}

    public default void setRobotOrentation(Pose2d pose) {}
}
