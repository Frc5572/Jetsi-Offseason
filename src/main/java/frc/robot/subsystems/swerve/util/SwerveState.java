package frc.robot.subsystems.swerve.util;

import java.util.NoSuchElementException;
import java.util.Optional;
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

/** State of the swerve drive */
public class SwerveState {

    /** State of the swerve drive */
    public SwerveState(SwerveModulePosition[] wheelPositions) {
        this.lastWheelPositions = wheelPositions;

        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
        }
    }

    private static final double poseBufferSizeSec = 2.0;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private static final Matrix<N3, N1> odometryStateStdDevs =
        new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    private SwerveModulePosition[] lastWheelPositions =
        new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()};

    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();

    // Assume gyro starts at zero
    private Rotation2d gyroOffset = Rotation2d.kZero;

    /** Update from the modules and gyro */
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

    /** Update from photonvision (or other vision solution) */
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

        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3x3 vision matrix
        var r = new double[] {translationStdDev * translationStdDev,
            translationStdDev * translationStdDev, rotationStdDev * rotationStdDev};
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
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

    public void addVisionObservation(CameraConstants camera, PhotonPipelineResult pipelineResult) {
        // TODO
    }

    /** Get current best estimate of the swerve's global position */
    public Pose2d getGlobalPoseEstimate() {
        return estimatedPose;
    }

}
