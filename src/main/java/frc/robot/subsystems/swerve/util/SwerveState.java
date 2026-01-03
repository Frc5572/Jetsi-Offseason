package frc.robot.subsystems.swerve.util;

import java.util.NoSuchElementException;
import java.util.Optional;
import org.jspecify.annotations.NullMarked;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.vision.CameraConstants;

/**
 * Maintains and updates the robot's estimated global pose for a swerve drive by fusing wheel
 * odometry, gyro measurements, and delayed vision updates.
 *
 * <p>
 * This class tracks two related poses:
 * <ul>
 * <li><b>Odometry pose</b> - derived purely from wheel encoder deltas and gyro yaw</li>
 * <li><b>Estimated pose</b> - a filtered pose that incorporates vision corrections</li>
 * </ul>
 *
 * <p>
 * A time-interpolated pose buffer is maintained to allow vision measurements with latency to be
 * applied retroactively at the correct timestamp.
 *
 * <p>
 * This implementation resembles a lightweight pose estimator rather than a full Kalman filter,
 * using configurable standard deviations to weight vision updates against odometry uncertainty.
 */
@NullMarked
public class SwerveState {

    /** Whether the pose estimator has been initialized from vision */
    private boolean initted = false;

    /**
     * Creates a new swerve state estimator.
     *
     * @param wheelPositions the initial swerve module positions used to seed odometry
     */
    public SwerveState(SwerveModulePosition[] wheelPositions) {
        this.lastWheelPositions = wheelPositions;

        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
        }
    }

    /** Length of time (in seconds) to retain pose history for vision replay */
    private static final double poseBufferSizeSec = 2.0;

    /**
     * Time-indexed buffer of past odometry poses used to compensate for vision processing latency.
     */
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    /** Standard deviations of odometry state (x, y, theta) */
    private static final Matrix<N3, N1> odometryStateStdDevs =
        new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

    /** Squared odometry state variances */
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    /** Last recorded swerve module positions for computing deltas */
    private SwerveModulePosition[] lastWheelPositions =
        new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()};

    /** Pose obtained from pure odometry integration */
    private Pose2d odometryPose = new Pose2d();
    /** Best current estimate of the robot pose after sensor fusion */
    private Pose2d estimatedPose = new Pose2d();

    /** Offset applied to gyro yaw to align it with field coordinates */
    private Rotation2d gyroOffset = Rotation2d.kZero;

    /**
     * Updates odometry and pose estimates using swerve module encoders and an optional gyro
     * measurement.
     *
     * @param wheelPositions current swerve module positions
     * @param gyroYaw current robot yaw, if available
     * @param timestamp measurement timestamp in seconds
     */
    public void addOdometryObservation(SwerveModulePosition[] wheelPositions,
        Optional<Rotation2d> gyroYaw, double timestamp) {
        Twist2d twist =
            Constants.Swerve.swerveKinematics.toTwist2d(lastWheelPositions, wheelPositions);
        lastWheelPositions = wheelPositions;
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist);

        gyroYaw.ifPresent(gyroAngle -> {
            odometryPose = new Pose2d(odometryPose.getTranslation(), gyroAngle.plus(gyroOffset));
        });

        poseBuffer.addSample(timestamp, odometryPose);

        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    /** Kalman-like gain matrix used for vision updates */
    private final Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());

    /**
     * Applies a vision-based pose correction at a historical timestamp.
     *
     * <p>
     * The correction is weighted based on provided vision measurement uncertainties and the current
     * odometry uncertainty.
     */
    private void addVisionObservationImpl(Pose3d cameraPose, Pose2d sample,
        Transform3d robotToCamera, double translationStdDev, double rotationStdDev,
        double timestamp) {
        var sampleToOdometryTransform = new Transform2d(sample, odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample);

        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3x3 vision matrix
        var r = new double[] {translationStdDev * translationStdDev,
            translationStdDev * translationStdDev, rotationStdDev * rotationStdDev};
        for (int row = 0; row < 3; row++) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }

        Transform2d transform = new Transform2d(estimateAtTime, cameraPose.toPose2d());

        var kTimesTransform = visionK.times(VecBuilder.fill(transform.getX(), transform.getY(),
            transform.getRotation().getRadians()));
        var scaledTransform = new Transform2d(kTimesTransform.get(0, 0), kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    /**
     * Adds a vision measurement using an externally computed camera pose.
     *
     * <p>
     * If the measurement timestamp is outside the pose buffer window, the update is ignored.
     *
     * @param cameraPose estimated camera pose in field coordinates
     * @param robotToCamera transform from robot to camera frame
     * @param translationStdDev translation measurement standard deviation (meters)
     * @param rotationStdDev rotation measurement standard deviation (radians)
     * @param timestamp measurement timestamp in seconds
     */
    public void addVisionObservation(Pose3d cameraPose, Transform3d robotToCamera,
        double translationStdDev, double rotationStdDev, double timestamp) {
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > timestamp) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        var sample = poseBuffer.getSample(timestamp);
        if (sample.isEmpty()) {
            return;
        }

        addVisionObservationImpl(cameraPose, sample.get(), robotToCamera, translationStdDev,
            rotationStdDev, timestamp);
    }

    /**
     * Adds a vision measurement from PhotonVision.
     *
     * <p>
     * If the estimator has not yet been initialized, a valid multi-tag estimate will be used to
     * seed the global pose.
     *
     * @param camera camera configuration constants
     * @param pipelineResult latest PhotonVision pipeline result
     */
    public void addVisionObservation(CameraConstants camera, PhotonPipelineResult pipelineResult) {
        if (!pipelineResult.hasTargets()) {
            return;
        }

        double timestamp = pipelineResult.getTimestampSeconds();

        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > timestamp) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        var sample = poseBuffer.getSample(timestamp);
        if (sample.isEmpty()) {
            return;
        }

        var multiTag = pipelineResult.getMultiTagResult();
        if (!initted) {
            multiTag.ifPresent(multiTag_ -> {
                Transform3d best = multiTag_.estimatedPose.best;
                Pose3d cameraPose =
                    new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
                Pose3d robotPose = cameraPose.plus(camera.robotToCamera.inverse());
                estimatedPose = robotPose.toPose2d();
                initted = true;
            });
        } else {
            if (multiTag.isPresent()) {
                // Multi Tag

            } else {
                // Single Tag

            }
        }
    }

    /**
     * Returns the current best estimate of the robot's global field pose.
     *
     * @return estimated robot pose in field coordinates
     */
    public Pose2d getGlobalPoseEstimate() {
        return estimatedPose;
    }

}
