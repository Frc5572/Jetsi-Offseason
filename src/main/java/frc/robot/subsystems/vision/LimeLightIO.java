package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface LimeLightIO {

    @AutoLog
    public class LimeLightInputs {
        LimelightHelpers.PoseEstimate[] cameraPose = {new LimelightHelpers.PoseEstimate()};
        double[] latestCapture = {0.0};
    }

    public default void updateInputs(LimeLightInputs inputs) {}
}
