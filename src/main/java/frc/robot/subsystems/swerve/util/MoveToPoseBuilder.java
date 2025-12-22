package frc.robot.subsystems.swerve.util;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/** generated */
public class MoveToPoseBuilder {
    private final Swerve swerve;
    private final Consumer<ChassisSpeeds> robotRelativeConsumer;

    /** generated */
    public MoveToPoseBuilder(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer) {
        this.swerve = swerve;
        this.robotRelativeConsumer = robotRelativeConsumer;
    }

    /** generated */
    public BuilderState100000 target(Supplier<Pose2d> target) {
        return new BuilderState100000(swerve, robotRelativeConsumer, target);
    }

    /** generated */
    public BuilderState100000 target(Pose2d targetConst) {
        Supplier<Pose2d> target = () -> targetConst;
        return new BuilderState100000(swerve, robotRelativeConsumer, target);
    }

    /** generated */
    public BuilderState010000 autoRoutine(AutoRoutine autoRoutine) {
        return new BuilderState010000(swerve, robotRelativeConsumer, autoRoutine);
    }

    /** generated */
    public BuilderState001000 maxSpeed(DoubleSupplier maxSpeed) {
        return new BuilderState001000(swerve, robotRelativeConsumer, maxSpeed);
    }

    /** generated */
    public BuilderState001000 maxSpeed(double maxSpeedConst) {
        DoubleSupplier maxSpeed = () -> maxSpeedConst;
        return new BuilderState001000(swerve, robotRelativeConsumer, maxSpeed);
    }

    /** generated */
    public BuilderState000100 flipForRed(boolean flipForRed) {
        return new BuilderState000100(swerve, robotRelativeConsumer, flipForRed);
    }

    /** generated */
    public BuilderState000010 translationTolerance(double translationTolerance) {
        return new BuilderState000010(swerve, robotRelativeConsumer, translationTolerance);
    }

    /** generated */
    public BuilderState000001 rotationTolerance(double rotationTolerance) {
        return new BuilderState000001(swerve, robotRelativeConsumer, rotationTolerance);
    }

    /** generated */
    public static class BuilderState100000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;

        private BuilderState100000(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null,
                () -> Constants.Swerve.autoMaxSpeed, true, 0.5, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState110000 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState110000(swerve, robotRelativeConsumer, target, autoRoutine);
        }

        /** generated */
        public BuilderState101000 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState101000(swerve, robotRelativeConsumer, target, maxSpeed);
        }

        /** generated */
        public BuilderState101000 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState101000(swerve, robotRelativeConsumer, target, maxSpeed);
        }

        /** generated */
        public BuilderState100100 flipForRed(boolean flipForRed) {
            return new BuilderState100100(swerve, robotRelativeConsumer, target, flipForRed);
        }

        /** generated */
        public BuilderState100010 translationTolerance(double translationTolerance) {
            return new BuilderState100010(swerve, robotRelativeConsumer, target,
                translationTolerance);
        }

        /** generated */
        public BuilderState100001 rotationTolerance(double rotationTolerance) {
            return new BuilderState100001(swerve, robotRelativeConsumer, target, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState010000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;

        private BuilderState010000(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
        }

        /** generated */
        public BuilderState110000 target(Supplier<Pose2d> target) {
            return new BuilderState110000(swerve, robotRelativeConsumer, target, autoRoutine);
        }

        /** generated */
        public BuilderState110000 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState110000(swerve, robotRelativeConsumer, target, autoRoutine);
        }

        /** generated */
        public BuilderState011000 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState011000(swerve, robotRelativeConsumer, autoRoutine, maxSpeed);
        }

        /** generated */
        public BuilderState011000 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState011000(swerve, robotRelativeConsumer, autoRoutine, maxSpeed);
        }

        /** generated */
        public BuilderState010100 flipForRed(boolean flipForRed) {
            return new BuilderState010100(swerve, robotRelativeConsumer, autoRoutine, flipForRed);
        }

        /** generated */
        public BuilderState010010 translationTolerance(double translationTolerance) {
            return new BuilderState010010(swerve, robotRelativeConsumer, autoRoutine,
                translationTolerance);
        }

        /** generated */
        public BuilderState010001 rotationTolerance(double rotationTolerance) {
            return new BuilderState010001(swerve, robotRelativeConsumer, autoRoutine,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState110000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;

        private BuilderState110000(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine,
                () -> Constants.Swerve.autoMaxSpeed, true, 0.5, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111000 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState111000(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed);
        }

        /** generated */
        public BuilderState111000 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState111000(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed);
        }

        /** generated */
        public BuilderState110100 flipForRed(boolean flipForRed) {
            return new BuilderState110100(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed);
        }

        /** generated */
        public BuilderState110010 translationTolerance(double translationTolerance) {
            return new BuilderState110010(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance);
        }

        /** generated */
        public BuilderState110001 rotationTolerance(double rotationTolerance) {
            return new BuilderState110001(swerve, robotRelativeConsumer, target, autoRoutine,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState001000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeed;

        private BuilderState001000(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            DoubleSupplier maxSpeed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeed = maxSpeed;
        }

        /** generated */
        public BuilderState101000 target(Supplier<Pose2d> target) {
            return new BuilderState101000(swerve, robotRelativeConsumer, target, maxSpeed);
        }

        /** generated */
        public BuilderState101000 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState101000(swerve, robotRelativeConsumer, target, maxSpeed);
        }

        /** generated */
        public BuilderState011000 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState011000(swerve, robotRelativeConsumer, autoRoutine, maxSpeed);
        }

        /** generated */
        public BuilderState001100 flipForRed(boolean flipForRed) {
            return new BuilderState001100(swerve, robotRelativeConsumer, maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState001010 translationTolerance(double translationTolerance) {
            return new BuilderState001010(swerve, robotRelativeConsumer, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState001001 rotationTolerance(double rotationTolerance) {
            return new BuilderState001001(swerve, robotRelativeConsumer, maxSpeed,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState101000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final DoubleSupplier maxSpeed;

        private BuilderState101000(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, DoubleSupplier maxSpeed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.maxSpeed = maxSpeed;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null, maxSpeed, true, 0.5,
                Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111000 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState111000(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed);
        }

        /** generated */
        public BuilderState101100 flipForRed(boolean flipForRed) {
            return new BuilderState101100(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState101010 translationTolerance(double translationTolerance) {
            return new BuilderState101010(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState101001 rotationTolerance(double rotationTolerance) {
            return new BuilderState101001(swerve, robotRelativeConsumer, target, maxSpeed,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState011000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;

        private BuilderState011000(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, DoubleSupplier maxSpeed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
        }

        /** generated */
        public BuilderState111000 target(Supplier<Pose2d> target) {
            return new BuilderState111000(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed);
        }

        /** generated */
        public BuilderState111000 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState111000(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed);
        }

        /** generated */
        public BuilderState011100 flipForRed(boolean flipForRed) {
            return new BuilderState011100(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState011010 translationTolerance(double translationTolerance) {
            return new BuilderState011010(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState011001 rotationTolerance(double rotationTolerance) {
            return new BuilderState011001(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState111000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;

        private BuilderState111000(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, DoubleSupplier maxSpeed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                true, 0.5, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111100 flipForRed(boolean flipForRed) {
            return new BuilderState111100(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState111010 translationTolerance(double translationTolerance) {
            return new BuilderState111010(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance);
        }

        /** generated */
        public BuilderState111001 rotationTolerance(double rotationTolerance) {
            return new BuilderState111001(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState000100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;

        private BuilderState000100(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            boolean flipForRed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
        }

        /** generated */
        public BuilderState100100 target(Supplier<Pose2d> target) {
            return new BuilderState100100(swerve, robotRelativeConsumer, target, flipForRed);
        }

        /** generated */
        public BuilderState100100 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState100100(swerve, robotRelativeConsumer, target, flipForRed);
        }

        /** generated */
        public BuilderState010100 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState010100(swerve, robotRelativeConsumer, autoRoutine, flipForRed);
        }

        /** generated */
        public BuilderState001100 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState001100(swerve, robotRelativeConsumer, maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState001100 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState001100(swerve, robotRelativeConsumer, maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState000110 translationTolerance(double translationTolerance) {
            return new BuilderState000110(swerve, robotRelativeConsumer, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState000101 rotationTolerance(double rotationTolerance) {
            return new BuilderState000101(swerve, robotRelativeConsumer, flipForRed,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState100100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final boolean flipForRed;

        private BuilderState100100(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, boolean flipForRed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.flipForRed = flipForRed;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null,
                () -> Constants.Swerve.autoMaxSpeed, flipForRed, 0.5, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState110100 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState110100(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed);
        }

        /** generated */
        public BuilderState101100 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState101100(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState101100 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState101100(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState100110 translationTolerance(double translationTolerance) {
            return new BuilderState100110(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState100101 rotationTolerance(double rotationTolerance) {
            return new BuilderState100101(swerve, robotRelativeConsumer, target, flipForRed,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState010100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final boolean flipForRed;

        private BuilderState010100(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, boolean flipForRed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.flipForRed = flipForRed;
        }

        /** generated */
        public BuilderState110100 target(Supplier<Pose2d> target) {
            return new BuilderState110100(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed);
        }

        /** generated */
        public BuilderState110100 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState110100(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed);
        }

        /** generated */
        public BuilderState011100 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState011100(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState011100 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState011100(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState010110 translationTolerance(double translationTolerance) {
            return new BuilderState010110(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState010101 rotationTolerance(double rotationTolerance) {
            return new BuilderState010101(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState110100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final boolean flipForRed;

        private BuilderState110100(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, boolean flipForRed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.flipForRed = flipForRed;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine,
                () -> Constants.Swerve.autoMaxSpeed, flipForRed, 0.5, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111100 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState111100(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState111100 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState111100(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState110110 translationTolerance(double translationTolerance) {
            return new BuilderState110110(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState110101 rotationTolerance(double rotationTolerance) {
            return new BuilderState110101(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState001100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;

        private BuilderState001100(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            DoubleSupplier maxSpeed, boolean flipForRed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
        }

        /** generated */
        public BuilderState101100 target(Supplier<Pose2d> target) {
            return new BuilderState101100(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState101100 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState101100(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState011100 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState011100(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed);
        }

        /** generated */
        public BuilderState001110 translationTolerance(double translationTolerance) {
            return new BuilderState001110(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState001101 rotationTolerance(double rotationTolerance) {
            return new BuilderState001101(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState101100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;

        private BuilderState101100(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, DoubleSupplier maxSpeed, boolean flipForRed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null, maxSpeed, flipForRed,
                0.5, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111100 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState111100(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState101110 translationTolerance(double translationTolerance) {
            return new BuilderState101110(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState101101 rotationTolerance(double rotationTolerance) {
            return new BuilderState101101(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState011100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;

        private BuilderState011100(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, DoubleSupplier maxSpeed, boolean flipForRed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
        }

        /** generated */
        public BuilderState111100 target(Supplier<Pose2d> target) {
            return new BuilderState111100(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState111100 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState111100(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed);
        }

        /** generated */
        public BuilderState011110 translationTolerance(double translationTolerance) {
            return new BuilderState011110(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState011101 rotationTolerance(double rotationTolerance) {
            return new BuilderState011101(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState111100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;

        private BuilderState111100(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, DoubleSupplier maxSpeed,
            boolean flipForRed) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, 0.5, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111110 translationTolerance(double translationTolerance) {
            return new BuilderState111110(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState111101 rotationTolerance(double rotationTolerance) {
            return new BuilderState111101(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState000010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;

        private BuilderState000010(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public BuilderState100010 target(Supplier<Pose2d> target) {
            return new BuilderState100010(swerve, robotRelativeConsumer, target,
                translationTolerance);
        }

        /** generated */
        public BuilderState100010 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState100010(swerve, robotRelativeConsumer, target,
                translationTolerance);
        }

        /** generated */
        public BuilderState010010 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState010010(swerve, robotRelativeConsumer, autoRoutine,
                translationTolerance);
        }

        /** generated */
        public BuilderState001010 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState001010(swerve, robotRelativeConsumer, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState001010 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState001010(swerve, robotRelativeConsumer, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState000110 flipForRed(boolean flipForRed) {
            return new BuilderState000110(swerve, robotRelativeConsumer, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState000011 rotationTolerance(double rotationTolerance) {
            return new BuilderState000011(swerve, robotRelativeConsumer, translationTolerance,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState100010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final double translationTolerance;

        private BuilderState100010(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null,
                () -> Constants.Swerve.autoMaxSpeed, true, translationTolerance,
                Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState110010 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState110010(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance);
        }

        /** generated */
        public BuilderState101010 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState101010(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState101010 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState101010(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState100110 flipForRed(boolean flipForRed) {
            return new BuilderState100110(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState100011 rotationTolerance(double rotationTolerance) {
            return new BuilderState100011(swerve, robotRelativeConsumer, target,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState010010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final double translationTolerance;

        private BuilderState010010(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public BuilderState110010 target(Supplier<Pose2d> target) {
            return new BuilderState110010(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance);
        }

        /** generated */
        public BuilderState110010 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState110010(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance);
        }

        /** generated */
        public BuilderState011010 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState011010(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState011010 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState011010(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState010110 flipForRed(boolean flipForRed) {
            return new BuilderState010110(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState010011 rotationTolerance(double rotationTolerance) {
            return new BuilderState010011(swerve, robotRelativeConsumer, autoRoutine,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState110010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final double translationTolerance;

        private BuilderState110010(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine,
                () -> Constants.Swerve.autoMaxSpeed, true, translationTolerance,
                Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111010 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState111010(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance);
        }

        /** generated */
        public BuilderState111010 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState111010(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance);
        }

        /** generated */
        public BuilderState110110 flipForRed(boolean flipForRed) {
            return new BuilderState110110(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState110011 rotationTolerance(double rotationTolerance) {
            return new BuilderState110011(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState001010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeed;
        private final double translationTolerance;

        private BuilderState001010(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            DoubleSupplier maxSpeed, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeed = maxSpeed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public BuilderState101010 target(Supplier<Pose2d> target) {
            return new BuilderState101010(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState101010 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState101010(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState011010 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState011010(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance);
        }

        /** generated */
        public BuilderState001110 flipForRed(boolean flipForRed) {
            return new BuilderState001110(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState001011 rotationTolerance(double rotationTolerance) {
            return new BuilderState001011(swerve, robotRelativeConsumer, maxSpeed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState101010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final DoubleSupplier maxSpeed;
        private final double translationTolerance;

        private BuilderState101010(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, DoubleSupplier maxSpeed, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.maxSpeed = maxSpeed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null, maxSpeed, true,
                translationTolerance, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111010 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState111010(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance);
        }

        /** generated */
        public BuilderState101110 flipForRed(boolean flipForRed) {
            return new BuilderState101110(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState101011 rotationTolerance(double rotationTolerance) {
            return new BuilderState101011(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState011010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final double translationTolerance;

        private BuilderState011010(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, DoubleSupplier maxSpeed, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public BuilderState111010 target(Supplier<Pose2d> target) {
            return new BuilderState111010(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance);
        }

        /** generated */
        public BuilderState111010 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState111010(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance);
        }

        /** generated */
        public BuilderState011110 flipForRed(boolean flipForRed) {
            return new BuilderState011110(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState011011 rotationTolerance(double rotationTolerance) {
            return new BuilderState011011(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState111010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final double translationTolerance;

        private BuilderState111010(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, DoubleSupplier maxSpeed,
            double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                true, translationTolerance, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111110 flipForRed(boolean flipForRed) {
            return new BuilderState111110(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState111011 rotationTolerance(double rotationTolerance) {
            return new BuilderState111011(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState000110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState000110(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            boolean flipForRed, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public BuilderState100110 target(Supplier<Pose2d> target) {
            return new BuilderState100110(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState100110 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState100110(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState010110 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState010110(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState001110 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState001110(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState001110 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState001110(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance);
        }

        /** generated */
        public BuilderState000111 rotationTolerance(double rotationTolerance) {
            return new BuilderState000111(swerve, robotRelativeConsumer, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState100110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState100110(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, boolean flipForRed, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null,
                () -> Constants.Swerve.autoMaxSpeed, flipForRed, translationTolerance,
                Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState110110 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState110110(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState101110 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState101110(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState101110 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState101110(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState100111 rotationTolerance(double rotationTolerance) {
            return new BuilderState100111(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState010110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState010110(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, boolean flipForRed, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public BuilderState110110 target(Supplier<Pose2d> target) {
            return new BuilderState110110(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState110110 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState110110(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState011110 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState011110(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState011110 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState011110(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState010111 rotationTolerance(double rotationTolerance) {
            return new BuilderState010111(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState110110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState110110(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, boolean flipForRed,
            double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine,
                () -> Constants.Swerve.autoMaxSpeed, flipForRed, translationTolerance,
                Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111110 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState111110(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState111110 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState111110(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState110111 rotationTolerance(double rotationTolerance) {
            return new BuilderState110111(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState001110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState001110(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            DoubleSupplier maxSpeed, boolean flipForRed, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public BuilderState101110 target(Supplier<Pose2d> target) {
            return new BuilderState101110(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState101110 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState101110(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState011110 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState011110(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState001111 rotationTolerance(double rotationTolerance) {
            return new BuilderState001111(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState101110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState101110(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, DoubleSupplier maxSpeed, boolean flipForRed,
            double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null, maxSpeed, flipForRed,
                translationTolerance, Units.degreesToRadians(5));
        }

        /** generated */
        public BuilderState111110 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState111110(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState101111 rotationTolerance(double rotationTolerance) {
            return new BuilderState101111(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState011110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState011110(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, DoubleSupplier maxSpeed, boolean flipForRed,
            double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public BuilderState111110 target(Supplier<Pose2d> target) {
            return new BuilderState111110(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState111110 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState111110(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, translationTolerance);
        }

        /** generated */
        public BuilderState011111 rotationTolerance(double rotationTolerance) {
            return new BuilderState011111(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState111110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState111110(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, DoubleSupplier maxSpeed,
            boolean flipForRed, double translationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, Units.degreesToRadians(5));
        }

        /** generated */
        public MoveToPose rotationTolerance(double rotationTolerance) {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState000001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double rotationTolerance;

        private BuilderState000001(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState100001 target(Supplier<Pose2d> target) {
            return new BuilderState100001(swerve, robotRelativeConsumer, target, rotationTolerance);
        }

        /** generated */
        public BuilderState100001 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState100001(swerve, robotRelativeConsumer, target, rotationTolerance);
        }

        /** generated */
        public BuilderState010001 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState010001(swerve, robotRelativeConsumer, autoRoutine,
                rotationTolerance);
        }

        /** generated */
        public BuilderState001001 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState001001(swerve, robotRelativeConsumer, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState001001 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState001001(swerve, robotRelativeConsumer, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState000101 flipForRed(boolean flipForRed) {
            return new BuilderState000101(swerve, robotRelativeConsumer, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState000011 translationTolerance(double translationTolerance) {
            return new BuilderState000011(swerve, robotRelativeConsumer, translationTolerance,
                rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState100001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final double rotationTolerance;

        private BuilderState100001(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null,
                () -> Constants.Swerve.autoMaxSpeed, true, 0.5, rotationTolerance);
        }

        /** generated */
        public BuilderState110001 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState110001(swerve, robotRelativeConsumer, target, autoRoutine,
                rotationTolerance);
        }

        /** generated */
        public BuilderState101001 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState101001(swerve, robotRelativeConsumer, target, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState101001 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState101001(swerve, robotRelativeConsumer, target, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState100101 flipForRed(boolean flipForRed) {
            return new BuilderState100101(swerve, robotRelativeConsumer, target, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState100011 translationTolerance(double translationTolerance) {
            return new BuilderState100011(swerve, robotRelativeConsumer, target,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState010001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final double rotationTolerance;

        private BuilderState010001(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState110001 target(Supplier<Pose2d> target) {
            return new BuilderState110001(swerve, robotRelativeConsumer, target, autoRoutine,
                rotationTolerance);
        }

        /** generated */
        public BuilderState110001 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState110001(swerve, robotRelativeConsumer, target, autoRoutine,
                rotationTolerance);
        }

        /** generated */
        public BuilderState011001 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState011001(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState011001 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState011001(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState010101 flipForRed(boolean flipForRed) {
            return new BuilderState010101(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState010011 translationTolerance(double translationTolerance) {
            return new BuilderState010011(swerve, robotRelativeConsumer, autoRoutine,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState110001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final double rotationTolerance;

        private BuilderState110001(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine,
                () -> Constants.Swerve.autoMaxSpeed, true, 0.5, rotationTolerance);
        }

        /** generated */
        public BuilderState111001 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState111001(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, rotationTolerance);
        }

        /** generated */
        public BuilderState111001 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState111001(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, rotationTolerance);
        }

        /** generated */
        public BuilderState110101 flipForRed(boolean flipForRed) {
            return new BuilderState110101(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState110011 translationTolerance(double translationTolerance) {
            return new BuilderState110011(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState001001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeed;
        private final double rotationTolerance;

        private BuilderState001001(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            DoubleSupplier maxSpeed, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeed = maxSpeed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState101001 target(Supplier<Pose2d> target) {
            return new BuilderState101001(swerve, robotRelativeConsumer, target, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState101001 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState101001(swerve, robotRelativeConsumer, target, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState011001 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState011001(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState001101 flipForRed(boolean flipForRed) {
            return new BuilderState001101(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState001011 translationTolerance(double translationTolerance) {
            return new BuilderState001011(swerve, robotRelativeConsumer, maxSpeed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState101001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final DoubleSupplier maxSpeed;
        private final double rotationTolerance;

        private BuilderState101001(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, DoubleSupplier maxSpeed, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.maxSpeed = maxSpeed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null, maxSpeed, true, 0.5,
                rotationTolerance);
        }

        /** generated */
        public BuilderState111001 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState111001(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, rotationTolerance);
        }

        /** generated */
        public BuilderState101101 flipForRed(boolean flipForRed) {
            return new BuilderState101101(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState101011 translationTolerance(double translationTolerance) {
            return new BuilderState101011(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState011001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final double rotationTolerance;

        private BuilderState011001(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, DoubleSupplier maxSpeed, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState111001 target(Supplier<Pose2d> target) {
            return new BuilderState111001(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, rotationTolerance);
        }

        /** generated */
        public BuilderState111001 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState111001(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, rotationTolerance);
        }

        /** generated */
        public BuilderState011101 flipForRed(boolean flipForRed) {
            return new BuilderState011101(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState011011 translationTolerance(double translationTolerance) {
            return new BuilderState011011(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState111001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final double rotationTolerance;

        private BuilderState111001(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, DoubleSupplier maxSpeed,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                true, 0.5, rotationTolerance);
        }

        /** generated */
        public BuilderState111101 flipForRed(boolean flipForRed) {
            return new BuilderState111101(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState111011 translationTolerance(double translationTolerance) {
            return new BuilderState111011(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState000101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState000101(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            boolean flipForRed, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState100101 target(Supplier<Pose2d> target) {
            return new BuilderState100101(swerve, robotRelativeConsumer, target, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState100101 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState100101(swerve, robotRelativeConsumer, target, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState010101 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState010101(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState001101 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState001101(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState001101 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState001101(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                rotationTolerance);
        }

        /** generated */
        public BuilderState000111 translationTolerance(double translationTolerance) {
            return new BuilderState000111(swerve, robotRelativeConsumer, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState100101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState100101(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, boolean flipForRed, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null,
                () -> Constants.Swerve.autoMaxSpeed, flipForRed, 0.5, rotationTolerance);
        }

        /** generated */
        public BuilderState110101 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState110101(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState101101 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState101101(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState101101 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState101101(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState100111 translationTolerance(double translationTolerance) {
            return new BuilderState100111(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState010101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState010101(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, boolean flipForRed, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState110101 target(Supplier<Pose2d> target) {
            return new BuilderState110101(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState110101 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState110101(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState011101 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState011101(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState011101 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState011101(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState010111 translationTolerance(double translationTolerance) {
            return new BuilderState010111(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState110101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState110101(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, boolean flipForRed,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine,
                () -> Constants.Swerve.autoMaxSpeed, flipForRed, 0.5, rotationTolerance);
        }

        /** generated */
        public BuilderState111101 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState111101(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState111101 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState111101(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState110111 translationTolerance(double translationTolerance) {
            return new BuilderState110111(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState001101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState001101(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            DoubleSupplier maxSpeed, boolean flipForRed, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState101101 target(Supplier<Pose2d> target) {
            return new BuilderState101101(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState101101 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState101101(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState011101 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState011101(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState001111 translationTolerance(double translationTolerance) {
            return new BuilderState001111(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState101101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState101101(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, DoubleSupplier maxSpeed, boolean flipForRed,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null, maxSpeed, flipForRed,
                0.5, rotationTolerance);
        }

        /** generated */
        public BuilderState111101 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState111101(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState101111 translationTolerance(double translationTolerance) {
            return new BuilderState101111(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState011101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState011101(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, DoubleSupplier maxSpeed, boolean flipForRed,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState111101 target(Supplier<Pose2d> target) {
            return new BuilderState111101(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState111101 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState111101(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, flipForRed, rotationTolerance);
        }

        /** generated */
        public BuilderState011111 translationTolerance(double translationTolerance) {
            return new BuilderState011111(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState111101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState111101(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, DoubleSupplier maxSpeed,
            boolean flipForRed, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, 0.5, rotationTolerance);
        }

        /** generated */
        public MoveToPose translationTolerance(double translationTolerance) {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState000011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState000011(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState100011 target(Supplier<Pose2d> target) {
            return new BuilderState100011(swerve, robotRelativeConsumer, target,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState100011 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState100011(swerve, robotRelativeConsumer, target,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState010011 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState010011(swerve, robotRelativeConsumer, autoRoutine,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState001011 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState001011(swerve, robotRelativeConsumer, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState001011 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState001011(swerve, robotRelativeConsumer, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState000111 flipForRed(boolean flipForRed) {
            return new BuilderState000111(swerve, robotRelativeConsumer, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState100011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState100011(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null,
                () -> Constants.Swerve.autoMaxSpeed, true, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState110011 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState110011(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState101011 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState101011(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState101011 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState101011(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState100111 flipForRed(boolean flipForRed) {
            return new BuilderState100111(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState010011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState010011(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState110011 target(Supplier<Pose2d> target) {
            return new BuilderState110011(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState110011 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState110011(swerve, robotRelativeConsumer, target, autoRoutine,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState011011 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState011011(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState011011 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState011011(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState010111 flipForRed(boolean flipForRed) {
            return new BuilderState010111(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState110011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState110011(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, double translationTolerance,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine,
                () -> Constants.Swerve.autoMaxSpeed, true, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState111011 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState111011(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState111011 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState111011(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState110111 flipForRed(boolean flipForRed) {
            return new BuilderState110111(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState001011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState001011(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            DoubleSupplier maxSpeed, double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeed = maxSpeed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState101011 target(Supplier<Pose2d> target) {
            return new BuilderState101011(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState101011 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState101011(swerve, robotRelativeConsumer, target, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState011011 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState011011(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState001111 flipForRed(boolean flipForRed) {
            return new BuilderState001111(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState101011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final DoubleSupplier maxSpeed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState101011(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, DoubleSupplier maxSpeed, double translationTolerance,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.maxSpeed = maxSpeed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null, maxSpeed, true,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState111011 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState111011(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState101111 flipForRed(boolean flipForRed) {
            return new BuilderState101111(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState011011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState011011(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, DoubleSupplier maxSpeed, double translationTolerance,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState111011 target(Supplier<Pose2d> target) {
            return new BuilderState111011(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState111011 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState111011(swerve, robotRelativeConsumer, target, autoRoutine,
                maxSpeed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState011111 flipForRed(boolean flipForRed) {
            return new BuilderState011111(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState111011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState111011(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, DoubleSupplier maxSpeed,
            double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                true, translationTolerance, rotationTolerance);
        }

        /** generated */
        public MoveToPose flipForRed(boolean flipForRed) {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState000111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState000111(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            boolean flipForRed, double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState100111 target(Supplier<Pose2d> target) {
            return new BuilderState100111(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState100111 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState100111(swerve, robotRelativeConsumer, target, flipForRed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState010111 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState010111(swerve, robotRelativeConsumer, autoRoutine, flipForRed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState001111 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState001111(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState001111 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState001111(swerve, robotRelativeConsumer, maxSpeed, flipForRed,
                translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState100111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState100111(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, boolean flipForRed, double translationTolerance,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null,
                () -> Constants.Swerve.autoMaxSpeed, flipForRed, translationTolerance,
                rotationTolerance);
        }

        /** generated */
        public BuilderState110111 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState110111(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState101111 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState101111(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState101111 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState101111(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState010111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState010111(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, boolean flipForRed, double translationTolerance,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState110111 target(Supplier<Pose2d> target) {
            return new BuilderState110111(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState110111 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState110111(swerve, robotRelativeConsumer, target, autoRoutine,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState011111 maxSpeed(DoubleSupplier maxSpeed) {
            return new BuilderState011111(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState011111 maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new BuilderState011111(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState110111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final AutoRoutine autoRoutine;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState110111(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, AutoRoutine autoRoutine, boolean flipForRed,
            double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.autoRoutine = autoRoutine;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine,
                () -> Constants.Swerve.autoMaxSpeed, flipForRed, translationTolerance,
                rotationTolerance);
        }

        /** generated */
        public MoveToPose maxSpeed(DoubleSupplier maxSpeed) {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public MoveToPose maxSpeed(double maxSpeedConst) {
            DoubleSupplier maxSpeed = () -> maxSpeedConst;
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState001111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState001111(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            DoubleSupplier maxSpeed, boolean flipForRed, double translationTolerance,
            double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public BuilderState101111 target(Supplier<Pose2d> target) {
            return new BuilderState101111(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState101111 target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new BuilderState101111(swerve, robotRelativeConsumer, target, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public BuilderState011111 autoRoutine(AutoRoutine autoRoutine) {
            return new BuilderState011111(swerve, robotRelativeConsumer, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState101111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> target;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState101111(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            Supplier<Pose2d> target, DoubleSupplier maxSpeed, boolean flipForRed,
            double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.target = target;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose finish() {
            return new MoveToPose(swerve, robotRelativeConsumer, target, null, maxSpeed, flipForRed,
                translationTolerance, rotationTolerance);
        }

        /** generated */
        public MoveToPose autoRoutine(AutoRoutine autoRoutine) {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }

    /** generated */
    public static class BuilderState011111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final AutoRoutine autoRoutine;
        private final DoubleSupplier maxSpeed;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState011111(Swerve swerve, Consumer<ChassisSpeeds> robotRelativeConsumer,
            AutoRoutine autoRoutine, DoubleSupplier maxSpeed, boolean flipForRed,
            double translationTolerance, double rotationTolerance) {
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.autoRoutine = autoRoutine;
            this.maxSpeed = maxSpeed;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        /** generated */
        public MoveToPose target(Supplier<Pose2d> target) {
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }

        /** generated */
        public MoveToPose target(Pose2d targetConst) {
            Supplier<Pose2d> target = () -> targetConst;
            return new MoveToPose(swerve, robotRelativeConsumer, target, autoRoutine, maxSpeed,
                flipForRed, translationTolerance, rotationTolerance);
        }
    }
}
