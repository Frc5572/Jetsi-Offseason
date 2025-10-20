package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.DrivetrainState.SwerveModuleStates;

public interface ExternalDrivetrainStateSolver {

    public void init(Pose2d pose, SwerveModuleStates states, double gyroYaw);

    public void resetPose(Pose2d pose, SwerveModuleStates states, double gyroYaw);

}
