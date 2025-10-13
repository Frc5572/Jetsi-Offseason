package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.LimelightHelpers;

/**
 * Singleton class to track Robot State
 */
public class RobotState {
    private boolean isInitialized = false;
    private AngularVelocity gyroRate = DegreesPerSecond.of(0.0);

    private final TimeInterpolatableBuffer<Rotation2d> rotationBuffer =
        TimeInterpolatableBuffer.createBuffer(1.5);

    public RobotState() {}

    private SwerveDrivePoseEstimator swerveOdometry = null;

    /**
     * Initialize this {@link RobotState}. Should only be called once (usually from the
     * {@link Swerve} constructor).
     */
    public void init(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, gyroYaw,
            positions, new Pose2d(0, 0, Rotation2d.fromDegrees(120)));
        rotationBuffer.clear();
        isInitialized = false;
        SmartDashboard.putNumber("cameraOffset", 0.0);
    }


    private double visionCutoff = 0;

    /**
     * Use prior information to set the pose. Should only be used at the start of the program, or
     * start of individual autonomous routines.
     */
    public void resetPose(Pose2d pose, SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.resetPosition(gyroYaw, positions, pose);
        visionCutoff = Timer.getFPGATimestamp();
        rotationBuffer.clear();
        rotationBuffer.getInternalBuffer().clear();
    }


    /**
     * Get the current pose estimate using the global solver.
     */
    public Pose2d getGlobalPoseEstimate() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void addSwerveObservation(SwerveModulePosition[] positions, Rotation2d gyroYaw) {
        swerveOdometry.update(gyroYaw, positions);
    }

    public void setGyroRate(AngularVelocity rate) {
        gyroRate = rate;
    }

    public void addVisionObservations(Pose2d pose, int tagCount, double timestamp) {
        var rejectUpdate = false;
        if (Math.abs(gyroRate.in(DegreesPerSecond)) > 720) {
            rejectUpdate = true;
        }
        if (tagCount == 0) {
            rejectUpdate = true;
        }
        if (!rejectUpdate) {
            swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            swerveOdometry.addVisionMeasurement(pose, timestamp);
        }

    }
}
