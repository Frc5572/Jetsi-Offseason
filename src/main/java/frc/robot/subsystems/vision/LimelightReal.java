package frc.robot.subsystems.vision;

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
            if (DriverStation.getAlliance().equals(Alliance.Blue)) {
                inputs.cameraPose[i] =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLNames[i]);
            } else if (DriverStation.getAlliance().equals(Alliance.Red)) {
                inputs.cameraPose[i] =
                    LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LLNames[i]);
            }
        }
    }
}
