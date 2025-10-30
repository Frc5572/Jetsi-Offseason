package frc.robot.util.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class DrivetrainState {

    private final SwerveDrivePoseEstimator swerveOdometry;

    private boolean isInitialized = false;

    public DrivetrainState(SwerveModulePosition[] positions, Rotation2d gyroYaw, double timestamp) {
        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.SWERVE_KINEMATICS, gyroYaw,
            positions, new Pose2d());
    }

    private double visionCutoff = 0.0;

    public void resetPose(Pose2d pose, SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.resetPosition(gyroYaw, positions, pose);
        visionCutoff = Timer.getFPGATimestamp();
    }

    public Pose2d getGlobalEstimate() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void addSwerveObservation(SwerveModulePosition[] positions, Rotation2d gyroYaw,
        double timestamp) {
        swerveOdometry.updateWithTime(timestamp, gyroYaw, positions);
    }

    // TODO vision stuff

}
