package frc.robot.subsystems.quest;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** QuestNav Subsystem */
public class Quest extends SubsystemBase {
    private QuestIO io;
    private QuestInputsAutoLogged inputs = new QuestInputsAutoLogged();
    private Pose2d questPose = new Pose2d();
    private Transform2d bot = new Transform2d(0.0, 0.0, Rotation2d.kZero);

    public Quest(QuestIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        calcPose();
        Logger.recordOutput("Quest/Pose", getPose());
    }

    private void calcPose() {
        if (inputs.poseFrames.length > 0) {
            questPose = inputs.poseFrames[inputs.poseFrames.length - 1].questPose3d().toPose2d();
        }
    }

    public Pose2d getPose() {
        return questPose.transformBy(bot);
    }

}
