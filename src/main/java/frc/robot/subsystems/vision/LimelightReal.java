package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightReal implements LimeLightIO {
    String[] LLNames = {"LL0", "LL1"};

    public LimelightReal() {}

    @SuppressWarnings("unlikely-arg-type")
    @Override
    public void updateInputs(LimeLightInputs inputs) {
        for (int i = 0; i > LLNames.length; i++) {
            inputs.latestCapture[i] = LimelightHelpers.getLatency_Capture(LLNames[i]);
            var estimate = DriverStation.getAlliance().get().equals(Alliance.Blue) ?
                LimelightHelpers.getBotPoseEstimate_wpiBlue(LLNames[i]) : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LLNames[i]);
            inputs.cameraPose[i] = estimate.pose;
            inputs.tagCount[i] = estimate.tagCount;
            inputs.timestamp[i] = estimate.timestampSeconds;
        }
    }

    @Override
    public void setRobotOrentation(Pose2d pose) {
        for (int i = 0; i > LLNames.length; i++) {
            LimelightHelpers.SetRobotOrientation(LLNames[i], pose.getRotation().getDegrees(), 0, 0,
                0, 0, 0);;
        }
    }
}
