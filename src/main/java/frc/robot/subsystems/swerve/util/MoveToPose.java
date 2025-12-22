package frc.robot.subsystems.swerve.util;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/** Drive Swerve to a given pose. */
public class MoveToPose extends Command {

    private final EventLoop eventLoop;

    private final Swerve swerve;
    private final Consumer<ChassisSpeeds> robotRelativeConsumer;
    private final Supplier<Pose2d> pose2dSupplier;
    private final DoubleSupplier maxSpeedSupplier;
    private final boolean flipForRed;
    private final double translationTolerance;
    private final double rotationTolerance;

    private boolean isActive = false;
    private boolean isCompleted = false;

    /** DO NOT USE THIS. Use {@link Swerve.moveToPose()} instead. */
    public MoveToPose(EventLoop eventLoop, Swerve swerve,
        Consumer<ChassisSpeeds> robotRelativeConsumer, Supplier<Pose2d> pose2dSupplier,
        DoubleSupplier maxSpeedSupplier, boolean flipForRed, double tol, double rTol) {
        this.eventLoop = eventLoop;
        this.swerve = swerve;
        this.robotRelativeConsumer = robotRelativeConsumer;
        this.pose2dSupplier = pose2dSupplier;
        this.maxSpeedSupplier = maxSpeedSupplier;
        this.flipForRed = flipForRed;
        this.translationTolerance = tol;
        this.rotationTolerance = rTol;
    }

    public Trigger active() {
        return new Trigger(eventLoop, () -> this.isActive);
    }

    public Trigger done() {
        return new Trigger(eventLoop, () -> this.isCompleted);
    }

    @Override
    public void initialize() {
        isActive = true;
        isCompleted = false;
    }

    private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(Constants.SwerveTransformPID.PID_XKP,
            Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
        new PIDController(Constants.SwerveTransformPID.PID_YKP,
            Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD),
        new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
            Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
            new Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));

    private Pose2d target = Pose2d.kZero;

    @Override
    public void execute() {
        target = this.pose2dSupplier.get();
        if (flipForRed) {
            // TODO game specific flipping
        }
        ChassisSpeeds ctrlEffort = holonomicDriveController
            .calculate(swerve.state.getGlobalPoseEstimate(), target, 0, target.getRotation());
        this.robotRelativeConsumer.accept(ctrlEffort);
    }

    @Override
    public void end(boolean interrupted) {
        isActive = false;
        isCompleted = !interrupted;
    }

    @Override
    public boolean isFinished() {
        Pose2d poseError = Pose2d.kZero.plus(target.minus(swerve.state.getGlobalPoseEstimate()));
        final var eTranslate = poseError.getTranslation();
        final var eRotate = poseError.getRotation();
        return Math.abs(eTranslate.getX()) < translationTolerance
            && Math.abs(eTranslate.getY()) < translationTolerance
            && Math.abs(eRotate.getDegrees()) < rotationTolerance;
    }

}
