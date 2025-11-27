package frc.robot.subsystems.quest;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.PoseFrame;

public interface QuestIO {

    @AutoLog
    class QuestInputs {
        PoseFrame[] poseFrames;
        int battery = 0;
        boolean connected = false;
        boolean tracking = false;

    }

    public default void updateInputs(QuestInputs inputs) {}

    public default void setPose(Pose2d pose) {}
}
