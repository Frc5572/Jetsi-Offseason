package frc.robot.subsystems.swerve.util;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

public class MoveToPoseBuilder {

    public static BuilderState10000000 eventLoop(EventLoop eventLoop) {
        return new BuilderState10000000(eventLoop);
    }

    public static BuilderState01000000 swerve(Swerve swerve) {
        return new BuilderState01000000(swerve);
    }

    public static BuilderState00100000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
        return new BuilderState00100000(robotRelativeConsumer);
    }

    public static BuilderState00010000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
        return new BuilderState00010000(pose2dSupplier);
    }
    public static class BuilderState10000000 {
        private final EventLoop eventLoop;

        private BuilderState10000000(EventLoop eventLoop){
            this.eventLoop = eventLoop;
        }

        public BuilderState11000000 swerve(Swerve swerve) {
            return new BuilderState11000000(eventLoop, swerve);
        }

        public BuilderState10100000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10100000(eventLoop, robotRelativeConsumer);
        }

        public BuilderState10010000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10010000(eventLoop, pose2dSupplier);
        }
    }
    public static class BuilderState01000000 {
        private final Swerve swerve;

        private BuilderState01000000(Swerve swerve){
            this.swerve = swerve;
        }

        public BuilderState11000000 eventLoop(EventLoop eventLoop) {
            return new BuilderState11000000(eventLoop, swerve);
        }

        public BuilderState01100000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01100000(swerve, robotRelativeConsumer);
        }

        public BuilderState01010000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01010000(swerve, pose2dSupplier);
        }
    }
    public static class BuilderState11000000 {
        private final EventLoop eventLoop;
        private final Swerve swerve;

        private BuilderState11000000(EventLoop eventLoop,Swerve swerve){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
        }

        public BuilderState11100000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11100000(eventLoop, swerve, robotRelativeConsumer);
        }

        public BuilderState11010000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11010000(eventLoop, swerve, pose2dSupplier);
        }
    }
    public static class BuilderState00100000 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;

        private BuilderState00100000(Consumer<ChassisSpeeds> robotRelativeConsumer){
            this.robotRelativeConsumer = robotRelativeConsumer;
        }

        public BuilderState10100000 eventLoop(EventLoop eventLoop) {
            return new BuilderState10100000(eventLoop, robotRelativeConsumer);
        }

        public BuilderState01100000 swerve(Swerve swerve) {
            return new BuilderState01100000(swerve, robotRelativeConsumer);
        }

        public BuilderState00110000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00110000(robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState10100000 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;

        private BuilderState10100000(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
        }

        public BuilderState11100000 swerve(Swerve swerve) {
            return new BuilderState11100000(eventLoop, swerve, robotRelativeConsumer);
        }

        public BuilderState10110000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10110000(eventLoop, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState01100000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;

        private BuilderState01100000(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
        }

        public BuilderState11100000 eventLoop(EventLoop eventLoop) {
            return new BuilderState11100000(eventLoop, swerve, robotRelativeConsumer);
        }

        public BuilderState01110000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01110000(swerve, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState11100000 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;

        private BuilderState11100000(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
        }

        public BuilderState11110000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11110000(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState00010000 {
        private final Supplier<Pose2d> pose2dSupplier;

        private BuilderState00010000(Supplier<Pose2d> pose2dSupplier){
            this.pose2dSupplier = pose2dSupplier;
        }

        public BuilderState10010000 eventLoop(EventLoop eventLoop) {
            return new BuilderState10010000(eventLoop, pose2dSupplier);
        }

        public BuilderState01010000 swerve(Swerve swerve) {
            return new BuilderState01010000(swerve, pose2dSupplier);
        }

        public BuilderState00110000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00110000(robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState10010000 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;

        private BuilderState10010000(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
        }

        public BuilderState11010000 swerve(Swerve swerve) {
            return new BuilderState11010000(eventLoop, swerve, pose2dSupplier);
        }

        public BuilderState10110000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10110000(eventLoop, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState01010000 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;

        private BuilderState01010000(Swerve swerve,Supplier<Pose2d> pose2dSupplier){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
        }

        public BuilderState11010000 eventLoop(EventLoop eventLoop) {
            return new BuilderState11010000(eventLoop, swerve, pose2dSupplier);
        }

        public BuilderState01110000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01110000(swerve, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState11010000 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;

        private BuilderState11010000(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
        }

        public BuilderState11110000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11110000(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState00110000 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;

        private BuilderState00110000(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
        }

        public BuilderState10110000 eventLoop(EventLoop eventLoop) {
            return new BuilderState10110000(eventLoop, robotRelativeConsumer, pose2dSupplier);
        }

        public BuilderState01110000 swerve(Swerve swerve) {
            return new BuilderState01110000(swerve, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState10110000 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;

        private BuilderState10110000(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
        }

        public BuilderState11110000 swerve(Swerve swerve) {
            return new BuilderState11110000(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState01110000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;

        private BuilderState01110000(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
        }

        public BuilderState11110000 eventLoop(EventLoop eventLoop) {
            return new BuilderState11110000(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier);
        }
    }
    public static class BuilderState11110000 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;

        private BuilderState11110000(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, () -> Constants.Swerve.autoMaxSpeed, true, 0.5, Units.degreesToRadians(5));
        }
    }
    public static class BuilderState00001000 {
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState00001000(DoubleSupplier maxSpeedSupplier){
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState10001000 eventLoop(EventLoop eventLoop) {
            return new BuilderState10001000(eventLoop, maxSpeedSupplier);
        }

        public BuilderState01001000 swerve(Swerve swerve) {
            return new BuilderState01001000(swerve, maxSpeedSupplier);
        }

        public BuilderState00101000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00101000(robotRelativeConsumer, maxSpeedSupplier);
        }

        public BuilderState00011000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00011000(pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState10001000 {
        private final EventLoop eventLoop;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState10001000(EventLoop eventLoop,DoubleSupplier maxSpeedSupplier){
            this.eventLoop = eventLoop;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11001000 swerve(Swerve swerve) {
            return new BuilderState11001000(eventLoop, swerve, maxSpeedSupplier);
        }

        public BuilderState10101000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10101000(eventLoop, robotRelativeConsumer, maxSpeedSupplier);
        }

        public BuilderState10011000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10011000(eventLoop, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState01001000 {
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState01001000(Swerve swerve,DoubleSupplier maxSpeedSupplier){
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11001000 eventLoop(EventLoop eventLoop) {
            return new BuilderState11001000(eventLoop, swerve, maxSpeedSupplier);
        }

        public BuilderState01101000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01101000(swerve, robotRelativeConsumer, maxSpeedSupplier);
        }

        public BuilderState01011000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01011000(swerve, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState11001000 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState11001000(EventLoop eventLoop,Swerve swerve,DoubleSupplier maxSpeedSupplier){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11101000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11101000(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier);
        }

        public BuilderState11011000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11011000(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState00101000 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState00101000(Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState10101000 eventLoop(EventLoop eventLoop) {
            return new BuilderState10101000(eventLoop, robotRelativeConsumer, maxSpeedSupplier);
        }

        public BuilderState01101000 swerve(Swerve swerve) {
            return new BuilderState01101000(swerve, robotRelativeConsumer, maxSpeedSupplier);
        }

        public BuilderState00111000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00111000(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState10101000 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState10101000(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11101000 swerve(Swerve swerve) {
            return new BuilderState11101000(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier);
        }

        public BuilderState10111000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10111000(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState01101000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState01101000(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11101000 eventLoop(EventLoop eventLoop) {
            return new BuilderState11101000(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier);
        }

        public BuilderState01111000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01111000(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState11101000 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState11101000(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11111000 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11111000(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState00011000 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState00011000(Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier){
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState10011000 eventLoop(EventLoop eventLoop) {
            return new BuilderState10011000(eventLoop, pose2dSupplier, maxSpeedSupplier);
        }

        public BuilderState01011000 swerve(Swerve swerve) {
            return new BuilderState01011000(swerve, pose2dSupplier, maxSpeedSupplier);
        }

        public BuilderState00111000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00111000(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState10011000 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState10011000(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11011000 swerve(Swerve swerve) {
            return new BuilderState11011000(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier);
        }

        public BuilderState10111000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10111000(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState01011000 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState01011000(Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11011000 eventLoop(EventLoop eventLoop) {
            return new BuilderState11011000(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier);
        }

        public BuilderState01111000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01111000(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState11011000 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState11011000(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11111000 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11111000(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState00111000 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState00111000(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState10111000 eventLoop(EventLoop eventLoop) {
            return new BuilderState10111000(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }

        public BuilderState01111000 swerve(Swerve swerve) {
            return new BuilderState01111000(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState10111000 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState10111000(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11111000 swerve(Swerve swerve) {
            return new BuilderState11111000(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState01111000 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState01111000(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public BuilderState11111000 eventLoop(EventLoop eventLoop) {
            return new BuilderState11111000(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier);
        }
    }
    public static class BuilderState11111000 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;

        private BuilderState11111000(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, true, 0.5, Units.degreesToRadians(5));
        }
    }
    public static class BuilderState00000100 {
        private final boolean flipForRed;

        private BuilderState00000100(boolean flipForRed){
            this.flipForRed = flipForRed;
        }

        public BuilderState10000100 eventLoop(EventLoop eventLoop) {
            return new BuilderState10000100(eventLoop, flipForRed);
        }

        public BuilderState01000100 swerve(Swerve swerve) {
            return new BuilderState01000100(swerve, flipForRed);
        }

        public BuilderState00100100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00100100(robotRelativeConsumer, flipForRed);
        }

        public BuilderState00010100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00010100(pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState10000100 {
        private final EventLoop eventLoop;
        private final boolean flipForRed;

        private BuilderState10000100(EventLoop eventLoop,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.flipForRed = flipForRed;
        }

        public BuilderState11000100 swerve(Swerve swerve) {
            return new BuilderState11000100(eventLoop, swerve, flipForRed);
        }

        public BuilderState10100100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10100100(eventLoop, robotRelativeConsumer, flipForRed);
        }

        public BuilderState10010100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10010100(eventLoop, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState01000100 {
        private final Swerve swerve;
        private final boolean flipForRed;

        private BuilderState01000100(Swerve swerve,boolean flipForRed){
            this.swerve = swerve;
            this.flipForRed = flipForRed;
        }

        public BuilderState11000100 eventLoop(EventLoop eventLoop) {
            return new BuilderState11000100(eventLoop, swerve, flipForRed);
        }

        public BuilderState01100100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01100100(swerve, robotRelativeConsumer, flipForRed);
        }

        public BuilderState01010100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01010100(swerve, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState11000100 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final boolean flipForRed;

        private BuilderState11000100(EventLoop eventLoop,Swerve swerve,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.flipForRed = flipForRed;
        }

        public BuilderState11100100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11100100(eventLoop, swerve, robotRelativeConsumer, flipForRed);
        }

        public BuilderState11010100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11010100(eventLoop, swerve, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState00100100 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;

        private BuilderState00100100(Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
        }

        public BuilderState10100100 eventLoop(EventLoop eventLoop) {
            return new BuilderState10100100(eventLoop, robotRelativeConsumer, flipForRed);
        }

        public BuilderState01100100 swerve(Swerve swerve) {
            return new BuilderState01100100(swerve, robotRelativeConsumer, flipForRed);
        }

        public BuilderState00110100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00110100(robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState10100100 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;

        private BuilderState10100100(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
        }

        public BuilderState11100100 swerve(Swerve swerve) {
            return new BuilderState11100100(eventLoop, swerve, robotRelativeConsumer, flipForRed);
        }

        public BuilderState10110100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10110100(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState01100100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;

        private BuilderState01100100(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
        }

        public BuilderState11100100 eventLoop(EventLoop eventLoop) {
            return new BuilderState11100100(eventLoop, swerve, robotRelativeConsumer, flipForRed);
        }

        public BuilderState01110100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01110100(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState11100100 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;

        private BuilderState11100100(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
        }

        public BuilderState11110100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11110100(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState00010100 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;

        private BuilderState00010100(Supplier<Pose2d> pose2dSupplier,boolean flipForRed){
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState10010100 eventLoop(EventLoop eventLoop) {
            return new BuilderState10010100(eventLoop, pose2dSupplier, flipForRed);
        }

        public BuilderState01010100 swerve(Swerve swerve) {
            return new BuilderState01010100(swerve, pose2dSupplier, flipForRed);
        }

        public BuilderState00110100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00110100(robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState10010100 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;

        private BuilderState10010100(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11010100 swerve(Swerve swerve) {
            return new BuilderState11010100(eventLoop, swerve, pose2dSupplier, flipForRed);
        }

        public BuilderState10110100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10110100(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState01010100 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;

        private BuilderState01010100(Swerve swerve,Supplier<Pose2d> pose2dSupplier,boolean flipForRed){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11010100 eventLoop(EventLoop eventLoop) {
            return new BuilderState11010100(eventLoop, swerve, pose2dSupplier, flipForRed);
        }

        public BuilderState01110100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01110100(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState11010100 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;

        private BuilderState11010100(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11110100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11110100(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState00110100 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;

        private BuilderState00110100(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState10110100 eventLoop(EventLoop eventLoop) {
            return new BuilderState10110100(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }

        public BuilderState01110100 swerve(Swerve swerve) {
            return new BuilderState01110100(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState10110100 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;

        private BuilderState10110100(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11110100 swerve(Swerve swerve) {
            return new BuilderState11110100(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState01110100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;

        private BuilderState01110100(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11110100 eventLoop(EventLoop eventLoop) {
            return new BuilderState11110100(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed);
        }
    }
    public static class BuilderState11110100 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;

        private BuilderState11110100(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, () -> Constants.Swerve.autoMaxSpeed, flipForRed, 0.5, Units.degreesToRadians(5));
        }
    }
    public static class BuilderState00001100 {
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState00001100(DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState10001100 eventLoop(EventLoop eventLoop) {
            return new BuilderState10001100(eventLoop, maxSpeedSupplier, flipForRed);
        }

        public BuilderState01001100 swerve(Swerve swerve) {
            return new BuilderState01001100(swerve, maxSpeedSupplier, flipForRed);
        }

        public BuilderState00101100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00101100(robotRelativeConsumer, maxSpeedSupplier, flipForRed);
        }

        public BuilderState00011100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00011100(pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState10001100 {
        private final EventLoop eventLoop;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState10001100(EventLoop eventLoop,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11001100 swerve(Swerve swerve) {
            return new BuilderState11001100(eventLoop, swerve, maxSpeedSupplier, flipForRed);
        }

        public BuilderState10101100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10101100(eventLoop, robotRelativeConsumer, maxSpeedSupplier, flipForRed);
        }

        public BuilderState10011100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10011100(eventLoop, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState01001100 {
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState01001100(Swerve swerve,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11001100 eventLoop(EventLoop eventLoop) {
            return new BuilderState11001100(eventLoop, swerve, maxSpeedSupplier, flipForRed);
        }

        public BuilderState01101100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01101100(swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed);
        }

        public BuilderState01011100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01011100(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState11001100 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState11001100(EventLoop eventLoop,Swerve swerve,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11101100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11101100(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed);
        }

        public BuilderState11011100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11011100(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState00101100 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState00101100(Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState10101100 eventLoop(EventLoop eventLoop) {
            return new BuilderState10101100(eventLoop, robotRelativeConsumer, maxSpeedSupplier, flipForRed);
        }

        public BuilderState01101100 swerve(Swerve swerve) {
            return new BuilderState01101100(swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed);
        }

        public BuilderState00111100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00111100(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState10101100 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState10101100(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11101100 swerve(Swerve swerve) {
            return new BuilderState11101100(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed);
        }

        public BuilderState10111100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10111100(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState01101100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState01101100(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11101100 eventLoop(EventLoop eventLoop) {
            return new BuilderState11101100(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed);
        }

        public BuilderState01111100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01111100(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState11101100 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState11101100(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11111100 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11111100(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState00011100 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState00011100(Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState10011100 eventLoop(EventLoop eventLoop) {
            return new BuilderState10011100(eventLoop, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }

        public BuilderState01011100 swerve(Swerve swerve) {
            return new BuilderState01011100(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }

        public BuilderState00111100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00111100(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState10011100 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState10011100(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11011100 swerve(Swerve swerve) {
            return new BuilderState11011100(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }

        public BuilderState10111100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10111100(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState01011100 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState01011100(Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11011100 eventLoop(EventLoop eventLoop) {
            return new BuilderState11011100(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }

        public BuilderState01111100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01111100(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState11011100 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState11011100(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11111100 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11111100(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState00111100 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState00111100(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState10111100 eventLoop(EventLoop eventLoop) {
            return new BuilderState10111100(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }

        public BuilderState01111100 swerve(Swerve swerve) {
            return new BuilderState01111100(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState10111100 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState10111100(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11111100 swerve(Swerve swerve) {
            return new BuilderState11111100(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState01111100 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState01111100(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public BuilderState11111100 eventLoop(EventLoop eventLoop) {
            return new BuilderState11111100(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed);
        }
    }
    public static class BuilderState11111100 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;

        private BuilderState11111100(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, 0.5, Units.degreesToRadians(5));
        }
    }
    public static class BuilderState00000010 {
        private final double translationTolerance;

        private BuilderState00000010(double translationTolerance){
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10000010 eventLoop(EventLoop eventLoop) {
            return new BuilderState10000010(eventLoop, translationTolerance);
        }

        public BuilderState01000010 swerve(Swerve swerve) {
            return new BuilderState01000010(swerve, translationTolerance);
        }

        public BuilderState00100010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00100010(robotRelativeConsumer, translationTolerance);
        }

        public BuilderState00010010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00010010(pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState10000010 {
        private final EventLoop eventLoop;
        private final double translationTolerance;

        private BuilderState10000010(EventLoop eventLoop,double translationTolerance){
            this.eventLoop = eventLoop;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11000010 swerve(Swerve swerve) {
            return new BuilderState11000010(eventLoop, swerve, translationTolerance);
        }

        public BuilderState10100010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10100010(eventLoop, robotRelativeConsumer, translationTolerance);
        }

        public BuilderState10010010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10010010(eventLoop, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState01000010 {
        private final Swerve swerve;
        private final double translationTolerance;

        private BuilderState01000010(Swerve swerve,double translationTolerance){
            this.swerve = swerve;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11000010 eventLoop(EventLoop eventLoop) {
            return new BuilderState11000010(eventLoop, swerve, translationTolerance);
        }

        public BuilderState01100010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01100010(swerve, robotRelativeConsumer, translationTolerance);
        }

        public BuilderState01010010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01010010(swerve, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState11000010 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final double translationTolerance;

        private BuilderState11000010(EventLoop eventLoop,Swerve swerve,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11100010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11100010(eventLoop, swerve, robotRelativeConsumer, translationTolerance);
        }

        public BuilderState11010010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11010010(eventLoop, swerve, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState00100010 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;

        private BuilderState00100010(Consumer<ChassisSpeeds> robotRelativeConsumer,double translationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10100010 eventLoop(EventLoop eventLoop) {
            return new BuilderState10100010(eventLoop, robotRelativeConsumer, translationTolerance);
        }

        public BuilderState01100010 swerve(Swerve swerve) {
            return new BuilderState01100010(swerve, robotRelativeConsumer, translationTolerance);
        }

        public BuilderState00110010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00110010(robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState10100010 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;

        private BuilderState10100010(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,double translationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11100010 swerve(Swerve swerve) {
            return new BuilderState11100010(eventLoop, swerve, robotRelativeConsumer, translationTolerance);
        }

        public BuilderState10110010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10110010(eventLoop, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState01100010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;

        private BuilderState01100010(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,double translationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11100010 eventLoop(EventLoop eventLoop) {
            return new BuilderState11100010(eventLoop, swerve, robotRelativeConsumer, translationTolerance);
        }

        public BuilderState01110010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01110010(swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState11100010 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;

        private BuilderState11100010(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11110010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11110010(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState00010010 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;

        private BuilderState00010010(Supplier<Pose2d> pose2dSupplier,double translationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10010010 eventLoop(EventLoop eventLoop) {
            return new BuilderState10010010(eventLoop, pose2dSupplier, translationTolerance);
        }

        public BuilderState01010010 swerve(Swerve swerve) {
            return new BuilderState01010010(swerve, pose2dSupplier, translationTolerance);
        }

        public BuilderState00110010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00110010(robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState10010010 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;

        private BuilderState10010010(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11010010 swerve(Swerve swerve) {
            return new BuilderState11010010(eventLoop, swerve, pose2dSupplier, translationTolerance);
        }

        public BuilderState10110010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10110010(eventLoop, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState01010010 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;

        private BuilderState01010010(Swerve swerve,Supplier<Pose2d> pose2dSupplier,double translationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11010010 eventLoop(EventLoop eventLoop) {
            return new BuilderState11010010(eventLoop, swerve, pose2dSupplier, translationTolerance);
        }

        public BuilderState01110010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01110010(swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState11010010 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;

        private BuilderState11010010(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11110010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11110010(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState00110010 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;

        private BuilderState00110010(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double translationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10110010 eventLoop(EventLoop eventLoop) {
            return new BuilderState10110010(eventLoop, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }

        public BuilderState01110010 swerve(Swerve swerve) {
            return new BuilderState01110010(swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState10110010 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;

        private BuilderState10110010(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11110010 swerve(Swerve swerve) {
            return new BuilderState11110010(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState01110010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;

        private BuilderState01110010(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double translationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11110010 eventLoop(EventLoop eventLoop) {
            return new BuilderState11110010(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance);
        }
    }
    public static class BuilderState11110010 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;

        private BuilderState11110010(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, () -> Constants.Swerve.autoMaxSpeed, true, translationTolerance, Units.degreesToRadians(5));
        }
    }
    public static class BuilderState00001010 {
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState00001010(DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10001010 eventLoop(EventLoop eventLoop) {
            return new BuilderState10001010(eventLoop, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState01001010 swerve(Swerve swerve) {
            return new BuilderState01001010(swerve, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState00101010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00101010(robotRelativeConsumer, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState00011010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00011010(pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState10001010 {
        private final EventLoop eventLoop;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState10001010(EventLoop eventLoop,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11001010 swerve(Swerve swerve) {
            return new BuilderState11001010(eventLoop, swerve, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState10101010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10101010(eventLoop, robotRelativeConsumer, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState10011010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10011010(eventLoop, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState01001010 {
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState01001010(Swerve swerve,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11001010 eventLoop(EventLoop eventLoop) {
            return new BuilderState11001010(eventLoop, swerve, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState01101010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01101010(swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState01011010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01011010(swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState11001010 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState11001010(EventLoop eventLoop,Swerve swerve,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11101010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11101010(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState11011010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11011010(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState00101010 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState00101010(Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10101010 eventLoop(EventLoop eventLoop) {
            return new BuilderState10101010(eventLoop, robotRelativeConsumer, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState01101010 swerve(Swerve swerve) {
            return new BuilderState01101010(swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState00111010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00111010(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState10101010 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState10101010(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11101010 swerve(Swerve swerve) {
            return new BuilderState11101010(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState10111010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10111010(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState01101010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState01101010(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11101010 eventLoop(EventLoop eventLoop) {
            return new BuilderState11101010(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState01111010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01111010(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState11101010 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState11101010(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11111010 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11111010(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState00011010 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState00011010(Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10011010 eventLoop(EventLoop eventLoop) {
            return new BuilderState10011010(eventLoop, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState01011010 swerve(Swerve swerve) {
            return new BuilderState01011010(swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState00111010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00111010(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState10011010 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState10011010(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11011010 swerve(Swerve swerve) {
            return new BuilderState11011010(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState10111010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10111010(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState01011010 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState01011010(Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11011010 eventLoop(EventLoop eventLoop) {
            return new BuilderState11011010(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState01111010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01111010(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState11011010 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState11011010(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11111010 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11111010(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState00111010 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState00111010(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10111010 eventLoop(EventLoop eventLoop) {
            return new BuilderState10111010(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }

        public BuilderState01111010 swerve(Swerve swerve) {
            return new BuilderState01111010(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState10111010 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState10111010(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11111010 swerve(Swerve swerve) {
            return new BuilderState11111010(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState01111010 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState01111010(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11111010 eventLoop(EventLoop eventLoop) {
            return new BuilderState11111010(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance);
        }
    }
    public static class BuilderState11111010 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;

        private BuilderState11111010(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, true, translationTolerance, Units.degreesToRadians(5));
        }
    }
    public static class BuilderState00000110 {
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState00000110(boolean flipForRed,double translationTolerance){
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10000110 eventLoop(EventLoop eventLoop) {
            return new BuilderState10000110(eventLoop, flipForRed, translationTolerance);
        }

        public BuilderState01000110 swerve(Swerve swerve) {
            return new BuilderState01000110(swerve, flipForRed, translationTolerance);
        }

        public BuilderState00100110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00100110(robotRelativeConsumer, flipForRed, translationTolerance);
        }

        public BuilderState00010110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00010110(pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState10000110 {
        private final EventLoop eventLoop;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState10000110(EventLoop eventLoop,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11000110 swerve(Swerve swerve) {
            return new BuilderState11000110(eventLoop, swerve, flipForRed, translationTolerance);
        }

        public BuilderState10100110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10100110(eventLoop, robotRelativeConsumer, flipForRed, translationTolerance);
        }

        public BuilderState10010110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10010110(eventLoop, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState01000110 {
        private final Swerve swerve;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState01000110(Swerve swerve,boolean flipForRed,double translationTolerance){
            this.swerve = swerve;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11000110 eventLoop(EventLoop eventLoop) {
            return new BuilderState11000110(eventLoop, swerve, flipForRed, translationTolerance);
        }

        public BuilderState01100110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01100110(swerve, robotRelativeConsumer, flipForRed, translationTolerance);
        }

        public BuilderState01010110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01010110(swerve, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState11000110 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState11000110(EventLoop eventLoop,Swerve swerve,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11100110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11100110(eventLoop, swerve, robotRelativeConsumer, flipForRed, translationTolerance);
        }

        public BuilderState11010110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11010110(eventLoop, swerve, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState00100110 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState00100110(Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double translationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10100110 eventLoop(EventLoop eventLoop) {
            return new BuilderState10100110(eventLoop, robotRelativeConsumer, flipForRed, translationTolerance);
        }

        public BuilderState01100110 swerve(Swerve swerve) {
            return new BuilderState01100110(swerve, robotRelativeConsumer, flipForRed, translationTolerance);
        }

        public BuilderState00110110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00110110(robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState10100110 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState10100110(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11100110 swerve(Swerve swerve) {
            return new BuilderState11100110(eventLoop, swerve, robotRelativeConsumer, flipForRed, translationTolerance);
        }

        public BuilderState10110110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10110110(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState01100110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState01100110(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double translationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11100110 eventLoop(EventLoop eventLoop) {
            return new BuilderState11100110(eventLoop, swerve, robotRelativeConsumer, flipForRed, translationTolerance);
        }

        public BuilderState01110110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01110110(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState11100110 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState11100110(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11110110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11110110(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState00010110 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState00010110(Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10010110 eventLoop(EventLoop eventLoop) {
            return new BuilderState10010110(eventLoop, pose2dSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01010110 swerve(Swerve swerve) {
            return new BuilderState01010110(swerve, pose2dSupplier, flipForRed, translationTolerance);
        }

        public BuilderState00110110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00110110(robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState10010110 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState10010110(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11010110 swerve(Swerve swerve) {
            return new BuilderState11010110(eventLoop, swerve, pose2dSupplier, flipForRed, translationTolerance);
        }

        public BuilderState10110110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10110110(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState01010110 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState01010110(Swerve swerve,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11010110 eventLoop(EventLoop eventLoop) {
            return new BuilderState11010110(eventLoop, swerve, pose2dSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01110110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01110110(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState11010110 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState11010110(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11110110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11110110(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState00110110 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState00110110(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10110110 eventLoop(EventLoop eventLoop) {
            return new BuilderState10110110(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01110110 swerve(Swerve swerve) {
            return new BuilderState01110110(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState10110110 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState10110110(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11110110 swerve(Swerve swerve) {
            return new BuilderState11110110(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState01110110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState01110110(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11110110 eventLoop(EventLoop eventLoop) {
            return new BuilderState11110110(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState11110110 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState11110110(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, () -> Constants.Swerve.autoMaxSpeed, flipForRed, translationTolerance, Units.degreesToRadians(5));
        }
    }
    public static class BuilderState00001110 {
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState00001110(DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10001110 eventLoop(EventLoop eventLoop) {
            return new BuilderState10001110(eventLoop, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01001110 swerve(Swerve swerve) {
            return new BuilderState01001110(swerve, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState00101110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00101110(robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState00011110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00011110(pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState10001110 {
        private final EventLoop eventLoop;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState10001110(EventLoop eventLoop,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11001110 swerve(Swerve swerve) {
            return new BuilderState11001110(eventLoop, swerve, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState10101110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10101110(eventLoop, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState10011110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10011110(eventLoop, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState01001110 {
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState01001110(Swerve swerve,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11001110 eventLoop(EventLoop eventLoop) {
            return new BuilderState11001110(eventLoop, swerve, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01101110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01101110(swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01011110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01011110(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState11001110 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState11001110(EventLoop eventLoop,Swerve swerve,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11101110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11101110(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState11011110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11011110(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState00101110 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState00101110(Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10101110 eventLoop(EventLoop eventLoop) {
            return new BuilderState10101110(eventLoop, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01101110 swerve(Swerve swerve) {
            return new BuilderState01101110(swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState00111110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00111110(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState10101110 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState10101110(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11101110 swerve(Swerve swerve) {
            return new BuilderState11101110(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState10111110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10111110(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState01101110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState01101110(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11101110 eventLoop(EventLoop eventLoop) {
            return new BuilderState11101110(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01111110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01111110(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState11101110 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState11101110(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11111110 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11111110(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState00011110 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState00011110(Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10011110 eventLoop(EventLoop eventLoop) {
            return new BuilderState10011110(eventLoop, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01011110 swerve(Swerve swerve) {
            return new BuilderState01011110(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState00111110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00111110(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState10011110 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState10011110(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11011110 swerve(Swerve swerve) {
            return new BuilderState11011110(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState10111110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10111110(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState01011110 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState01011110(Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11011110 eventLoop(EventLoop eventLoop) {
            return new BuilderState11011110(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01111110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01111110(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState11011110 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState11011110(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11111110 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11111110(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState00111110 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState00111110(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState10111110 eventLoop(EventLoop eventLoop) {
            return new BuilderState10111110(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }

        public BuilderState01111110 swerve(Swerve swerve) {
            return new BuilderState01111110(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState10111110 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState10111110(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11111110 swerve(Swerve swerve) {
            return new BuilderState11111110(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState01111110 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState01111110(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public BuilderState11111110 eventLoop(EventLoop eventLoop) {
            return new BuilderState11111110(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance);
        }
    }
    public static class BuilderState11111110 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;

        private BuilderState11111110(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, Units.degreesToRadians(5));
        }
    }
    public static class BuilderState00000001 {
        private final double rotationTolerance;

        private BuilderState00000001(double rotationTolerance){
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10000001 eventLoop(EventLoop eventLoop) {
            return new BuilderState10000001(eventLoop, rotationTolerance);
        }

        public BuilderState01000001 swerve(Swerve swerve) {
            return new BuilderState01000001(swerve, rotationTolerance);
        }

        public BuilderState00100001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00100001(robotRelativeConsumer, rotationTolerance);
        }

        public BuilderState00010001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00010001(pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState10000001 {
        private final EventLoop eventLoop;
        private final double rotationTolerance;

        private BuilderState10000001(EventLoop eventLoop,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11000001 swerve(Swerve swerve) {
            return new BuilderState11000001(eventLoop, swerve, rotationTolerance);
        }

        public BuilderState10100001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10100001(eventLoop, robotRelativeConsumer, rotationTolerance);
        }

        public BuilderState10010001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10010001(eventLoop, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState01000001 {
        private final Swerve swerve;
        private final double rotationTolerance;

        private BuilderState01000001(Swerve swerve,double rotationTolerance){
            this.swerve = swerve;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11000001 eventLoop(EventLoop eventLoop) {
            return new BuilderState11000001(eventLoop, swerve, rotationTolerance);
        }

        public BuilderState01100001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01100001(swerve, robotRelativeConsumer, rotationTolerance);
        }

        public BuilderState01010001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01010001(swerve, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState11000001 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final double rotationTolerance;

        private BuilderState11000001(EventLoop eventLoop,Swerve swerve,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11100001(eventLoop, swerve, robotRelativeConsumer, rotationTolerance);
        }

        public BuilderState11010001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11010001(eventLoop, swerve, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState00100001 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double rotationTolerance;

        private BuilderState00100001(Consumer<ChassisSpeeds> robotRelativeConsumer,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10100001 eventLoop(EventLoop eventLoop) {
            return new BuilderState10100001(eventLoop, robotRelativeConsumer, rotationTolerance);
        }

        public BuilderState01100001 swerve(Swerve swerve) {
            return new BuilderState01100001(swerve, robotRelativeConsumer, rotationTolerance);
        }

        public BuilderState00110001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00110001(robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState10100001 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double rotationTolerance;

        private BuilderState10100001(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100001 swerve(Swerve swerve) {
            return new BuilderState11100001(eventLoop, swerve, robotRelativeConsumer, rotationTolerance);
        }

        public BuilderState10110001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10110001(eventLoop, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState01100001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double rotationTolerance;

        private BuilderState01100001(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100001 eventLoop(EventLoop eventLoop) {
            return new BuilderState11100001(eventLoop, swerve, robotRelativeConsumer, rotationTolerance);
        }

        public BuilderState01110001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01110001(swerve, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState11100001 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double rotationTolerance;

        private BuilderState11100001(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11110001(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState00010001 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final double rotationTolerance;

        private BuilderState00010001(Supplier<Pose2d> pose2dSupplier,double rotationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10010001 eventLoop(EventLoop eventLoop) {
            return new BuilderState10010001(eventLoop, pose2dSupplier, rotationTolerance);
        }

        public BuilderState01010001 swerve(Swerve swerve) {
            return new BuilderState01010001(swerve, pose2dSupplier, rotationTolerance);
        }

        public BuilderState00110001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00110001(robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState10010001 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double rotationTolerance;

        private BuilderState10010001(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11010001 swerve(Swerve swerve) {
            return new BuilderState11010001(eventLoop, swerve, pose2dSupplier, rotationTolerance);
        }

        public BuilderState10110001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10110001(eventLoop, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState01010001 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double rotationTolerance;

        private BuilderState01010001(Swerve swerve,Supplier<Pose2d> pose2dSupplier,double rotationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11010001 eventLoop(EventLoop eventLoop) {
            return new BuilderState11010001(eventLoop, swerve, pose2dSupplier, rotationTolerance);
        }

        public BuilderState01110001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01110001(swerve, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState11010001 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double rotationTolerance;

        private BuilderState11010001(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11110001(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState00110001 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double rotationTolerance;

        private BuilderState00110001(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10110001 eventLoop(EventLoop eventLoop) {
            return new BuilderState10110001(eventLoop, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }

        public BuilderState01110001 swerve(Swerve swerve) {
            return new BuilderState01110001(swerve, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState10110001 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double rotationTolerance;

        private BuilderState10110001(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110001 swerve(Swerve swerve) {
            return new BuilderState11110001(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState01110001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double rotationTolerance;

        private BuilderState01110001(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110001 eventLoop(EventLoop eventLoop) {
            return new BuilderState11110001(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, rotationTolerance);
        }
    }
    public static class BuilderState11110001 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double rotationTolerance;

        private BuilderState11110001(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, () -> Constants.Swerve.autoMaxSpeed, true, 0.5, rotationTolerance);
        }
    }
    public static class BuilderState00001001 {
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState00001001(DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10001001 eventLoop(EventLoop eventLoop) {
            return new BuilderState10001001(eventLoop, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState01001001 swerve(Swerve swerve) {
            return new BuilderState01001001(swerve, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState00101001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00101001(robotRelativeConsumer, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState00011001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00011001(pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState10001001 {
        private final EventLoop eventLoop;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState10001001(EventLoop eventLoop,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11001001 swerve(Swerve swerve) {
            return new BuilderState11001001(eventLoop, swerve, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState10101001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10101001(eventLoop, robotRelativeConsumer, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState10011001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10011001(eventLoop, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState01001001 {
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState01001001(Swerve swerve,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11001001 eventLoop(EventLoop eventLoop) {
            return new BuilderState11001001(eventLoop, swerve, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState01101001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01101001(swerve, robotRelativeConsumer, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState01011001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01011001(swerve, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState11001001 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState11001001(EventLoop eventLoop,Swerve swerve,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11101001(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState11011001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11011001(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState00101001 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState00101001(Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10101001 eventLoop(EventLoop eventLoop) {
            return new BuilderState10101001(eventLoop, robotRelativeConsumer, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState01101001 swerve(Swerve swerve) {
            return new BuilderState01101001(swerve, robotRelativeConsumer, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState00111001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00111001(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState10101001 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState10101001(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101001 swerve(Swerve swerve) {
            return new BuilderState11101001(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState10111001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10111001(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState01101001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState01101001(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101001 eventLoop(EventLoop eventLoop) {
            return new BuilderState11101001(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState01111001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01111001(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState11101001 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState11101001(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111001 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11111001(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState00011001 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState00011001(Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10011001 eventLoop(EventLoop eventLoop) {
            return new BuilderState10011001(eventLoop, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState01011001 swerve(Swerve swerve) {
            return new BuilderState01011001(swerve, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState00111001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00111001(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState10011001 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState10011001(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11011001 swerve(Swerve swerve) {
            return new BuilderState11011001(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState10111001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10111001(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState01011001 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState01011001(Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11011001 eventLoop(EventLoop eventLoop) {
            return new BuilderState11011001(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState01111001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01111001(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState11011001 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState11011001(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111001 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11111001(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState00111001 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState00111001(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10111001 eventLoop(EventLoop eventLoop) {
            return new BuilderState10111001(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }

        public BuilderState01111001 swerve(Swerve swerve) {
            return new BuilderState01111001(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState10111001 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState10111001(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111001 swerve(Swerve swerve) {
            return new BuilderState11111001(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState01111001 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState01111001(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111001 eventLoop(EventLoop eventLoop) {
            return new BuilderState11111001(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, rotationTolerance);
        }
    }
    public static class BuilderState11111001 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double rotationTolerance;

        private BuilderState11111001(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, true, 0.5, rotationTolerance);
        }
    }
    public static class BuilderState00000101 {
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState00000101(boolean flipForRed,double rotationTolerance){
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10000101 eventLoop(EventLoop eventLoop) {
            return new BuilderState10000101(eventLoop, flipForRed, rotationTolerance);
        }

        public BuilderState01000101 swerve(Swerve swerve) {
            return new BuilderState01000101(swerve, flipForRed, rotationTolerance);
        }

        public BuilderState00100101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00100101(robotRelativeConsumer, flipForRed, rotationTolerance);
        }

        public BuilderState00010101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00010101(pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState10000101 {
        private final EventLoop eventLoop;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState10000101(EventLoop eventLoop,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11000101 swerve(Swerve swerve) {
            return new BuilderState11000101(eventLoop, swerve, flipForRed, rotationTolerance);
        }

        public BuilderState10100101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10100101(eventLoop, robotRelativeConsumer, flipForRed, rotationTolerance);
        }

        public BuilderState10010101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10010101(eventLoop, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState01000101 {
        private final Swerve swerve;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState01000101(Swerve swerve,boolean flipForRed,double rotationTolerance){
            this.swerve = swerve;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11000101 eventLoop(EventLoop eventLoop) {
            return new BuilderState11000101(eventLoop, swerve, flipForRed, rotationTolerance);
        }

        public BuilderState01100101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01100101(swerve, robotRelativeConsumer, flipForRed, rotationTolerance);
        }

        public BuilderState01010101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01010101(swerve, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState11000101 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState11000101(EventLoop eventLoop,Swerve swerve,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11100101(eventLoop, swerve, robotRelativeConsumer, flipForRed, rotationTolerance);
        }

        public BuilderState11010101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11010101(eventLoop, swerve, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState00100101 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState00100101(Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10100101 eventLoop(EventLoop eventLoop) {
            return new BuilderState10100101(eventLoop, robotRelativeConsumer, flipForRed, rotationTolerance);
        }

        public BuilderState01100101 swerve(Swerve swerve) {
            return new BuilderState01100101(swerve, robotRelativeConsumer, flipForRed, rotationTolerance);
        }

        public BuilderState00110101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00110101(robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState10100101 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState10100101(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100101 swerve(Swerve swerve) {
            return new BuilderState11100101(eventLoop, swerve, robotRelativeConsumer, flipForRed, rotationTolerance);
        }

        public BuilderState10110101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10110101(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState01100101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState01100101(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100101 eventLoop(EventLoop eventLoop) {
            return new BuilderState11100101(eventLoop, swerve, robotRelativeConsumer, flipForRed, rotationTolerance);
        }

        public BuilderState01110101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01110101(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState11100101 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState11100101(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11110101(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState00010101 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState00010101(Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double rotationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10010101 eventLoop(EventLoop eventLoop) {
            return new BuilderState10010101(eventLoop, pose2dSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01010101 swerve(Swerve swerve) {
            return new BuilderState01010101(swerve, pose2dSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState00110101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00110101(robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState10010101 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState10010101(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11010101 swerve(Swerve swerve) {
            return new BuilderState11010101(eventLoop, swerve, pose2dSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState10110101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10110101(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState01010101 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState01010101(Swerve swerve,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double rotationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11010101 eventLoop(EventLoop eventLoop) {
            return new BuilderState11010101(eventLoop, swerve, pose2dSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01110101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01110101(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState11010101 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState11010101(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11110101(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState00110101 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState00110101(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10110101 eventLoop(EventLoop eventLoop) {
            return new BuilderState10110101(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01110101 swerve(Swerve swerve) {
            return new BuilderState01110101(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState10110101 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState10110101(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110101 swerve(Swerve swerve) {
            return new BuilderState11110101(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState01110101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState01110101(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110101 eventLoop(EventLoop eventLoop) {
            return new BuilderState11110101(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState11110101 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState11110101(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, () -> Constants.Swerve.autoMaxSpeed, flipForRed, 0.5, rotationTolerance);
        }
    }
    public static class BuilderState00001101 {
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState00001101(DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10001101 eventLoop(EventLoop eventLoop) {
            return new BuilderState10001101(eventLoop, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01001101 swerve(Swerve swerve) {
            return new BuilderState01001101(swerve, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState00101101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00101101(robotRelativeConsumer, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState00011101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00011101(pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState10001101 {
        private final EventLoop eventLoop;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState10001101(EventLoop eventLoop,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11001101 swerve(Swerve swerve) {
            return new BuilderState11001101(eventLoop, swerve, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState10101101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10101101(eventLoop, robotRelativeConsumer, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState10011101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10011101(eventLoop, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState01001101 {
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState01001101(Swerve swerve,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11001101 eventLoop(EventLoop eventLoop) {
            return new BuilderState11001101(eventLoop, swerve, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01101101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01101101(swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01011101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01011101(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState11001101 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState11001101(EventLoop eventLoop,Swerve swerve,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11101101(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState11011101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11011101(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState00101101 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState00101101(Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10101101 eventLoop(EventLoop eventLoop) {
            return new BuilderState10101101(eventLoop, robotRelativeConsumer, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01101101 swerve(Swerve swerve) {
            return new BuilderState01101101(swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState00111101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00111101(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState10101101 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState10101101(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101101 swerve(Swerve swerve) {
            return new BuilderState11101101(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState10111101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10111101(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState01101101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState01101101(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101101 eventLoop(EventLoop eventLoop) {
            return new BuilderState11101101(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01111101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01111101(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState11101101 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState11101101(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111101 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11111101(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState00011101 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState00011101(Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10011101 eventLoop(EventLoop eventLoop) {
            return new BuilderState10011101(eventLoop, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01011101 swerve(Swerve swerve) {
            return new BuilderState01011101(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState00111101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00111101(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState10011101 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState10011101(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11011101 swerve(Swerve swerve) {
            return new BuilderState11011101(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState10111101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10111101(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState01011101 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState01011101(Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11011101 eventLoop(EventLoop eventLoop) {
            return new BuilderState11011101(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01111101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01111101(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState11011101 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState11011101(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111101 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11111101(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState00111101 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState00111101(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10111101 eventLoop(EventLoop eventLoop) {
            return new BuilderState10111101(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }

        public BuilderState01111101 swerve(Swerve swerve) {
            return new BuilderState01111101(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState10111101 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState10111101(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111101 swerve(Swerve swerve) {
            return new BuilderState11111101(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState01111101 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState01111101(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111101 eventLoop(EventLoop eventLoop) {
            return new BuilderState11111101(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, rotationTolerance);
        }
    }
    public static class BuilderState11111101 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double rotationTolerance;

        private BuilderState11111101(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, 0.5, rotationTolerance);
        }
    }
    public static class BuilderState00000011 {
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00000011(double translationTolerance,double rotationTolerance){
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10000011 eventLoop(EventLoop eventLoop) {
            return new BuilderState10000011(eventLoop, translationTolerance, rotationTolerance);
        }

        public BuilderState01000011 swerve(Swerve swerve) {
            return new BuilderState01000011(swerve, translationTolerance, rotationTolerance);
        }

        public BuilderState00100011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00100011(robotRelativeConsumer, translationTolerance, rotationTolerance);
        }

        public BuilderState00010011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00010011(pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10000011 {
        private final EventLoop eventLoop;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10000011(EventLoop eventLoop,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11000011 swerve(Swerve swerve) {
            return new BuilderState11000011(eventLoop, swerve, translationTolerance, rotationTolerance);
        }

        public BuilderState10100011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10100011(eventLoop, robotRelativeConsumer, translationTolerance, rotationTolerance);
        }

        public BuilderState10010011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10010011(eventLoop, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01000011 {
        private final Swerve swerve;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01000011(Swerve swerve,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11000011 eventLoop(EventLoop eventLoop) {
            return new BuilderState11000011(eventLoop, swerve, translationTolerance, rotationTolerance);
        }

        public BuilderState01100011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01100011(swerve, robotRelativeConsumer, translationTolerance, rotationTolerance);
        }

        public BuilderState01010011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01010011(swerve, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11000011 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11000011(EventLoop eventLoop,Swerve swerve,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11100011(eventLoop, swerve, robotRelativeConsumer, translationTolerance, rotationTolerance);
        }

        public BuilderState11010011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11010011(eventLoop, swerve, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00100011 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00100011(Consumer<ChassisSpeeds> robotRelativeConsumer,double translationTolerance,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10100011 eventLoop(EventLoop eventLoop) {
            return new BuilderState10100011(eventLoop, robotRelativeConsumer, translationTolerance, rotationTolerance);
        }

        public BuilderState01100011 swerve(Swerve swerve) {
            return new BuilderState01100011(swerve, robotRelativeConsumer, translationTolerance, rotationTolerance);
        }

        public BuilderState00110011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00110011(robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10100011 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10100011(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100011 swerve(Swerve swerve) {
            return new BuilderState11100011(eventLoop, swerve, robotRelativeConsumer, translationTolerance, rotationTolerance);
        }

        public BuilderState10110011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10110011(eventLoop, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01100011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01100011(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100011 eventLoop(EventLoop eventLoop) {
            return new BuilderState11100011(eventLoop, swerve, robotRelativeConsumer, translationTolerance, rotationTolerance);
        }

        public BuilderState01110011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01110011(swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11100011 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11100011(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11110011(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00010011 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00010011(Supplier<Pose2d> pose2dSupplier,double translationTolerance,double rotationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10010011 eventLoop(EventLoop eventLoop) {
            return new BuilderState10010011(eventLoop, pose2dSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01010011 swerve(Swerve swerve) {
            return new BuilderState01010011(swerve, pose2dSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState00110011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00110011(robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10010011 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10010011(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11010011 swerve(Swerve swerve) {
            return new BuilderState11010011(eventLoop, swerve, pose2dSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState10110011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10110011(eventLoop, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01010011 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01010011(Swerve swerve,Supplier<Pose2d> pose2dSupplier,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11010011 eventLoop(EventLoop eventLoop) {
            return new BuilderState11010011(eventLoop, swerve, pose2dSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01110011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01110011(swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11010011 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11010011(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11110011(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00110011 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00110011(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double translationTolerance,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10110011 eventLoop(EventLoop eventLoop) {
            return new BuilderState10110011(eventLoop, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01110011 swerve(Swerve swerve) {
            return new BuilderState01110011(swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10110011 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10110011(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110011 swerve(Swerve swerve) {
            return new BuilderState11110011(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01110011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01110011(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110011 eventLoop(EventLoop eventLoop) {
            return new BuilderState11110011(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11110011 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11110011(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, () -> Constants.Swerve.autoMaxSpeed, true, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00001011 {
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00001011(DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10001011 eventLoop(EventLoop eventLoop) {
            return new BuilderState10001011(eventLoop, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01001011 swerve(Swerve swerve) {
            return new BuilderState01001011(swerve, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState00101011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00101011(robotRelativeConsumer, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState00011011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00011011(pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10001011 {
        private final EventLoop eventLoop;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10001011(EventLoop eventLoop,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11001011 swerve(Swerve swerve) {
            return new BuilderState11001011(eventLoop, swerve, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState10101011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10101011(eventLoop, robotRelativeConsumer, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState10011011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10011011(eventLoop, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01001011 {
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01001011(Swerve swerve,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11001011 eventLoop(EventLoop eventLoop) {
            return new BuilderState11001011(eventLoop, swerve, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01101011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01101011(swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01011011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01011011(swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11001011 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11001011(EventLoop eventLoop,Swerve swerve,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11101011(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState11011011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11011011(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00101011 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00101011(Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10101011 eventLoop(EventLoop eventLoop) {
            return new BuilderState10101011(eventLoop, robotRelativeConsumer, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01101011 swerve(Swerve swerve) {
            return new BuilderState01101011(swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState00111011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00111011(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10101011 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10101011(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101011 swerve(Swerve swerve) {
            return new BuilderState11101011(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState10111011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10111011(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01101011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01101011(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101011 eventLoop(EventLoop eventLoop) {
            return new BuilderState11101011(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01111011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01111011(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11101011 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11101011(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111011 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11111011(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00011011 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00011011(Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10011011 eventLoop(EventLoop eventLoop) {
            return new BuilderState10011011(eventLoop, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01011011 swerve(Swerve swerve) {
            return new BuilderState01011011(swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState00111011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00111011(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10011011 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10011011(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11011011 swerve(Swerve swerve) {
            return new BuilderState11011011(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState10111011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10111011(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01011011 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01011011(Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11011011 eventLoop(EventLoop eventLoop) {
            return new BuilderState11011011(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01111011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01111011(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11011011 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11011011(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111011 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11111011(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00111011 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00111011(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10111011 eventLoop(EventLoop eventLoop) {
            return new BuilderState10111011(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }

        public BuilderState01111011 swerve(Swerve swerve) {
            return new BuilderState01111011(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10111011 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10111011(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111011 swerve(Swerve swerve) {
            return new BuilderState11111011(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01111011 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01111011(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11111011 eventLoop(EventLoop eventLoop) {
            return new BuilderState11111011(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11111011 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11111011(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, true, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00000111 {
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00000111(boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10000111 eventLoop(EventLoop eventLoop) {
            return new BuilderState10000111(eventLoop, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01000111 swerve(Swerve swerve) {
            return new BuilderState01000111(swerve, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState00100111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00100111(robotRelativeConsumer, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState00010111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00010111(pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10000111 {
        private final EventLoop eventLoop;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10000111(EventLoop eventLoop,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11000111 swerve(Swerve swerve) {
            return new BuilderState11000111(eventLoop, swerve, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState10100111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10100111(eventLoop, robotRelativeConsumer, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState10010111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10010111(eventLoop, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01000111 {
        private final Swerve swerve;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01000111(Swerve swerve,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11000111 eventLoop(EventLoop eventLoop) {
            return new BuilderState11000111(eventLoop, swerve, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01100111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01100111(swerve, robotRelativeConsumer, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01010111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01010111(swerve, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11000111 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11000111(EventLoop eventLoop,Swerve swerve,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11100111(eventLoop, swerve, robotRelativeConsumer, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState11010111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11010111(eventLoop, swerve, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00100111 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00100111(Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10100111 eventLoop(EventLoop eventLoop) {
            return new BuilderState10100111(eventLoop, robotRelativeConsumer, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01100111 swerve(Swerve swerve) {
            return new BuilderState01100111(swerve, robotRelativeConsumer, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState00110111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00110111(robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10100111 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10100111(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100111 swerve(Swerve swerve) {
            return new BuilderState11100111(eventLoop, swerve, robotRelativeConsumer, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState10110111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10110111(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01100111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01100111(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11100111 eventLoop(EventLoop eventLoop) {
            return new BuilderState11100111(eventLoop, swerve, robotRelativeConsumer, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01110111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01110111(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11100111 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11100111(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11110111(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00010111 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00010111(Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10010111 eventLoop(EventLoop eventLoop) {
            return new BuilderState10010111(eventLoop, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01010111 swerve(Swerve swerve) {
            return new BuilderState01010111(swerve, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState00110111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00110111(robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10010111 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10010111(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11010111 swerve(Swerve swerve) {
            return new BuilderState11010111(eventLoop, swerve, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState10110111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10110111(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01010111 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01010111(Swerve swerve,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11010111 eventLoop(EventLoop eventLoop) {
            return new BuilderState11010111(eventLoop, swerve, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01110111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01110111(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11010111 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11010111(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11110111(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00110111 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00110111(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10110111 eventLoop(EventLoop eventLoop) {
            return new BuilderState10110111(eventLoop, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01110111 swerve(Swerve swerve) {
            return new BuilderState01110111(swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10110111 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10110111(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110111 swerve(Swerve swerve) {
            return new BuilderState11110111(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01110111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01110111(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11110111 eventLoop(EventLoop eventLoop) {
            return new BuilderState11110111(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11110111 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11110111(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose finish() {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, () -> Constants.Swerve.autoMaxSpeed, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00001111 {
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00001111(DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10001111 eventLoop(EventLoop eventLoop) {
            return new BuilderState10001111(eventLoop, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01001111 swerve(Swerve swerve) {
            return new BuilderState01001111(swerve, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState00101111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00101111(robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState00011111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00011111(pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10001111 {
        private final EventLoop eventLoop;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10001111(EventLoop eventLoop,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11001111 swerve(Swerve swerve) {
            return new BuilderState11001111(eventLoop, swerve, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState10101111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10101111(eventLoop, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState10011111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10011111(eventLoop, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01001111 {
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01001111(Swerve swerve,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11001111 eventLoop(EventLoop eventLoop) {
            return new BuilderState11001111(eventLoop, swerve, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01101111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01101111(swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01011111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01011111(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11001111 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11001111(EventLoop eventLoop,Swerve swerve,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState11101111(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState11011111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState11011111(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00101111 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00101111(Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10101111 eventLoop(EventLoop eventLoop) {
            return new BuilderState10101111(eventLoop, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01101111 swerve(Swerve swerve) {
            return new BuilderState01101111(swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState00111111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState00111111(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10101111 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10101111(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101111 swerve(Swerve swerve) {
            return new BuilderState11101111(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState10111111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState10111111(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01101111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01101111(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11101111 eventLoop(EventLoop eventLoop) {
            return new BuilderState11101111(eventLoop, swerve, robotRelativeConsumer, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01111111 pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new BuilderState01111111(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11101111 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11101111(EventLoop eventLoop,Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose pose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00011111 {
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00011111(Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10011111 eventLoop(EventLoop eventLoop) {
            return new BuilderState10011111(eventLoop, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01011111 swerve(Swerve swerve) {
            return new BuilderState01011111(swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState00111111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState00111111(robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10011111 {
        private final EventLoop eventLoop;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10011111(EventLoop eventLoop,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11011111 swerve(Swerve swerve) {
            return new BuilderState11011111(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState10111111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState10111111(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01011111 {
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01011111(Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState11011111 eventLoop(EventLoop eventLoop) {
            return new BuilderState11011111(eventLoop, swerve, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01111111 robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new BuilderState01111111(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState11011111 {
        private final EventLoop eventLoop;
        private final Swerve swerve;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState11011111(EventLoop eventLoop,Swerve swerve,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.swerve = swerve;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose robotRelativeConsumer(Consumer<ChassisSpeeds> robotRelativeConsumer) {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState00111111 {
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState00111111(Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public BuilderState10111111 eventLoop(EventLoop eventLoop) {
            return new BuilderState10111111(eventLoop, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }

        public BuilderState01111111 swerve(Swerve swerve) {
            return new BuilderState01111111(swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState10111111 {
        private final EventLoop eventLoop;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState10111111(EventLoop eventLoop,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.eventLoop = eventLoop;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose swerve(Swerve swerve) {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
    public static class BuilderState01111111 {
        private final Swerve swerve;
        private final Consumer<ChassisSpeeds> robotRelativeConsumer;
        private final Supplier<Pose2d> pose2dSupplier;
        private final DoubleSupplier maxSpeedSupplier;
        private final boolean flipForRed;
        private final double translationTolerance;
        private final double rotationTolerance;

        private BuilderState01111111(Swerve swerve,Consumer<ChassisSpeeds> robotRelativeConsumer,Supplier<Pose2d> pose2dSupplier,DoubleSupplier maxSpeedSupplier,boolean flipForRed,double translationTolerance,double rotationTolerance){
            this.swerve = swerve;
            this.robotRelativeConsumer = robotRelativeConsumer;
            this.pose2dSupplier = pose2dSupplier;
            this.maxSpeedSupplier = maxSpeedSupplier;
            this.flipForRed = flipForRed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }

        public MoveToPose eventLoop(EventLoop eventLoop) {
            return new MoveToPose(eventLoop, swerve, robotRelativeConsumer, pose2dSupplier, maxSpeedSupplier, flipForRed, translationTolerance, rotationTolerance);
        }
    }
}
