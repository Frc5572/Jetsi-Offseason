package frc.robot.viz;

import java.util.Arrays;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveSim;

@NullMarked
public class RobotViz {

    private final Supplier<Pose3d> robotPoseSupplier;

    public RobotViz(@Nullable SwerveSim sim, Swerve swerve) {
        if (sim != null) {
            robotPoseSupplier = () -> new Pose3d(sim.mapleSim.getSimulatedDriveTrainPose());
        } else {
            robotPoseSupplier = () -> new Pose3d(swerve.state.getGlobalPoseEstimate());
        }
    }

    public void periodic() {
        Pose3d robotPose = robotPoseSupplier.get();
        Logger.recordOutput("Viz/GlobalEstPose", robotPose);
        Logger.recordOutput("Viz/CameraPoses", Arrays.stream(Constants.Vision.cameraConstants)
            .map(consts -> robotPose.plus(consts.robotToCamera)).toArray(Pose3d[]::new));
    }

}
