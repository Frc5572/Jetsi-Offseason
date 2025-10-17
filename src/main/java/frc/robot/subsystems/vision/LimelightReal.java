package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightReal implements LimeLightIO {
  String[] LLNames = {"limelight-one", "limelight-two"};

  public LimelightReal() {}

  @Override
  public void updateInputs(LimeLightInputs inputs) {
    var estimateOne = DriverStation.getAlliance().get().equals(Alliance.Blue) ?
      LimelightHelpers.getBotPoseEstimate_wpiBlue(LLNames[0]) : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LLNames[0]);
    var estimateTwo = DriverStation.getAlliance().get().equals(Alliance.Blue) ?
      LimelightHelpers.getBotPoseEstimate_wpiBlue(LLNames[1]) : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LLNames[1]);

    if (estimateOne != null) {
      inputs.cameraPoseOne = estimateOne.pose;
      inputs.tagCountOne = estimateOne.tagCount;
      inputs.timestampOne = estimateOne.timestampSeconds;
    }

    if (estimateTwo != null) {
      inputs.cameraPoseTwo = estimateTwo.pose;
      inputs.tagCountTwo = estimateTwo.tagCount;
      inputs.timestampTwo = estimateTwo.timestampSeconds;
    }

    inputs.latestCaptureOne = LimelightHelpers.getLatency_Capture(LLNames[0]);
    inputs.latestCaptureTwo = LimelightHelpers.getLatency_Capture(LLNames[1]);
  }

  @Override
  public void setRobotOrentation(Pose2d pose) {
    for (int i = 0; i < LLNames.length; i++) {
      LimelightHelpers.SetRobotOrientation(LLNames[i], pose.getRotation().getDegrees(), 0, 0,
        0, 0, 0);
    }
  }
}
