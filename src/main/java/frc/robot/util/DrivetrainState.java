package frc.robot.util;

import java.util.stream.IntStream;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

/** Full SE2 state estimator for a swerve drive */
public class DrivetrainState {

    /** State of each swerve module in a SoA format */
    public static class SwerveModuleStates {
        /** Angles in rotations */
        public final double[] angles = new double[4];
        /** Distances in meters */
        public final double[] distances = new double[4];
    }

    private final SwerveDrivePoseEstimator swerveOdometry;
    private final ExternalDrivetrainStateSolver[] externalSolvers;

    /**
     * @param kinematics Kinematics of the whole drive
     * @param moduleStates State of each swerve module in SoA format
     * @param gyroYaw Yaw reading from gyro in rotations
     * @param initialPose Starting pose
     * @param externalSolvers offboard solvers to supplement state estimate
     */
    public DrivetrainState(SwerveDriveKinematics kinematics, SwerveModuleStates moduleStates,
        double gyroYaw, Pose2d initialPose, ExternalDrivetrainStateSolver... externalSolvers) {
        this.swerveOdometry = new SwerveDrivePoseEstimator(kinematics,
            Rotation2d.fromRotations(gyroYaw), fromSwerveModuleStates(moduleStates), initialPose);
        this.externalSolvers = externalSolvers;
        for (ExternalDrivetrainStateSolver solver : externalSolvers) {
            solver.init(initialPose, moduleStates, gyroYaw);
        }
    }

    private double visionCutoff = 0;

    /**
     * Use prior information to set the pose. Should only be used at the start of the program, or
     * start of individual autonomous routines.
     */
    public void resetPose(Pose2d pose, SwerveModuleStates moduleStates, double gyroYaw) {
        swerveOdometry.resetPosition(Rotation2d.fromRotations(gyroYaw),
            fromSwerveModuleStates(moduleStates), pose);
        visionCutoff = Timer.getFPGATimestamp();
        for (ExternalDrivetrainStateSolver solver : externalSolvers) {
            solver.init(pose, moduleStates, gyroYaw);
        }
    }

    /**
     * Get the current pose estimate using the global solver.
     */
    public Pose2d getGlobalPoseEstimate() {
        return swerveOdometry.getEstimatedPosition();
    }

    private static SwerveModulePosition[] fromSwerveModuleStates(SwerveModuleStates moduleStates) {
        return IntStream.range(0, 4).mapToObj((i) -> {
            return new SwerveModulePosition(moduleStates.distances[i],
                Rotation2d.fromRotations(moduleStates.angles[i]));
        }).toArray(DrivetrainState::backedArray);
    }

    private static final SwerveModulePosition[] backing1 = new SwerveModulePosition[4],
        backing2 = new SwerveModulePosition[4];
    private static boolean whichBacking = false;

    private static SwerveModulePosition[] backedArray(int count) {
        whichBacking = !whichBacking;
        if (whichBacking) {
            return backing1;
        } else {
            return backing2;
        }
    }

}
