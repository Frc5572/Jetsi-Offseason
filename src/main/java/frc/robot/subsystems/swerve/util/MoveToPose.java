package frc.robot.subsystems.swerve.util;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.typestate.AltMethod;
import frc.robot.util.typestate.InitField;
import frc.robot.util.typestate.OptionalField;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Drive Swerve to a given pose. */
public class MoveToPose extends Command {

    private final AutoRoutine autoRoutine;
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

    /** DO NOT USE THIS. Use Swerve.moveToPose instead. */
    @TypeStateBuilder("MoveToPoseBuilder")
    public MoveToPose(@InitField Swerve swerve,
        @InitField Consumer<ChassisSpeeds> robotRelativeConsumer,
        @RequiredField(alt = @AltMethod(type = Pose2d.class, parameter_name = "targetConst",
            value = "() -> targetConst")) Supplier<Pose2d> target,
        @OptionalField("null") AutoRoutine autoRoutine,
        @OptionalField(value = "() -> frc.robot.Constants.Swerve.autoMaxSpeed",
            alt = @AltMethod(type = double.class, parameter_name = "maxSpeedConst",
                value = "() -> maxSpeedConst")) DoubleSupplier maxSpeed,
        @OptionalField("true") boolean flipForRed,
        @OptionalField("0.5") double translationTolerance,
        @OptionalField("edu.wpi.first.math.util.Units.degreesToRadians(5)") double rotationTolerance) {
        this.autoRoutine = autoRoutine;
        if (autoRoutine == null) {
            this.eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
        } else {
            this.eventLoop = autoRoutine.loop();
        }
        this.swerve = swerve;
        this.robotRelativeConsumer = robotRelativeConsumer;
        this.pose2dSupplier = target;
        this.maxSpeedSupplier = maxSpeed;
        this.flipForRed = flipForRed;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationTolerance;
    }

    /**
     * Returns a trigger that is true while the trajectory is scheduled.
     *
     * @return A trigger that is true while the trajectory is scheduled.
     */
    public Trigger active() {
        if (autoRoutine != null) {
            return new Trigger(eventLoop,
                () -> this.isActive && autoRoutine.active().getAsBoolean());
        }
        return new Trigger(eventLoop, () -> this.isActive);
    }

    /**
     * Returns a trigger that is true when the trajectory is finished. This will return true on
     * subsequent evaluations, until this command is restarted.
     */
    public Trigger done() {
        return new Trigger(eventLoop, () -> this.isCompleted);
    }

    @Override
    public void initialize() {
        isActive = true;
        isCompleted = false;
    }

    private static final HolonomicDriveController holonomicDriveController =
        new HolonomicDriveController(new PIDController(Constants.SwerveTransformPID.translationP,
            Constants.SwerveTransformPID.translationI, Constants.SwerveTransformPID.translationD),
            new PIDController(Constants.SwerveTransformPID.translationP,
                Constants.SwerveTransformPID.translationI,
                Constants.SwerveTransformPID.translationD),
            new ProfiledPIDController(Constants.SwerveTransformPID.rotationP,
                Constants.SwerveTransformPID.rotationI, Constants.SwerveTransformPID.rotationD,
                new Constraints(Constants.SwerveTransformPID.maxAngularVelocity,
                    Constants.SwerveTransformPID.maxAngularAcceleration)));

    private Pose2d target = Pose2d.kZero;

    @Override
    public void execute() {
        target = this.pose2dSupplier.get();
        if (flipForRed) {
            target = AllianceFlipUtil.apply(target);
        }
        ChassisSpeeds ctrlEffort = holonomicDriveController
            .calculate(swerve.state.getGlobalPoseEstimate(), target, 0, target.getRotation());
        double speed = Math.hypot(ctrlEffort.vxMetersPerSecond, ctrlEffort.vyMetersPerSecond);
        double maxSpeed = this.maxSpeedSupplier.getAsDouble();
        if (speed > maxSpeed) {
            double mul = maxSpeed / speed;
            ctrlEffort.vxMetersPerSecond *= mul;
            ctrlEffort.vyMetersPerSecond *= mul;
        }
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
