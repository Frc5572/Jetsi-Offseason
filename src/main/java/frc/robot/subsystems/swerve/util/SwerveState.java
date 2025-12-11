package frc.robot.subsystems.swerve.util;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** State of the swerve drive */
public class SwerveState {

    /** State of the swerve drive */
    public SwerveState() {

    }

    /** Update from the modules and gyro */
    public void addOdometryObservation(SwerveModulePosition[] wheelPositions,
        Optional<Rotation2d> gyroYaw, double timestamp) {

    }

    /** Get current best estimate of the swerve's global position */
    public Pose2d getGlobalPoseEstimate() {
        return null;
    }

}
