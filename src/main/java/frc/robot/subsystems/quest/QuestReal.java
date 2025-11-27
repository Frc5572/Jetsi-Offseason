package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.QuestNav;

public class QuestReal implements QuestIO {
    private final QuestNav questNav = new QuestNav();

    public QuestReal() {}

    @Override
    public void updateInputs(QuestInputs inputs) {
        inputs.poseFrames = questNav.getAllUnreadPoseFrames();
        inputs.battery = questNav.getBatteryPercent().getAsInt();
        inputs.tracking = questNav.isTracking();
        inputs.connected = questNav.isConnected();
    }

    @Override
    public void setPose(Pose2d pose) {
        questNav.setPose(new Pose3d(pose));
    }
}
