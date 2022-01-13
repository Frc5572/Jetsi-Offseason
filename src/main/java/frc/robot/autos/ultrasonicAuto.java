package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Ultrasonic;

import java.util.List;

import javax.lang.model.element.ExecutableElement;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ExecUltrasonic;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.commands.moveNewMotor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.ExecUltrasonic;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.NewMotor;

public class ultrasonicAuto extends SequentialCommandGroup {
    Ultrasonic ultrasonic;
    // ExecUltrasonic c_ExecUltrasonic;

    public ultrasonicAuto(Swerve s_Swerve, Ultrasonic ultrasonic){
        // ultrasonic = new Ultrasonic();
        // c_ExecUltrasonic = new ExecUltrasonic();
        System.out.println("Ultrasonic Auto !!");
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory firstHalfTrajectory =
            TrajectoryGenerator.generateTrajectory(

                List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new Rotation2d(0)), new Pose2d(1, 1, new Rotation2d(0))),
                config);
        Trajectory secondHalfTrajectory =
            TrajectoryGenerator.generateTrajectory(

                List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new Rotation2d(0)), new Pose2d(1, 1, new Rotation2d(0))),
                config);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand firstHalfTraject = new SwerveControllerCommand(firstHalfTrajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics, new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);
        SwerveControllerCommand secondHalfTraject = new SwerveControllerCommand(secondHalfTrajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics, new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);
        WaitUntilCommand untilUltrasonicZero = new WaitUntilCommand(() -> ultrasonic.getDistanceValue() == 0);
        PrintCommand ultrasonicValue = new PrintCommand("Ultrasonic: " + ultrasonic.getDistanceValue() + "cm");
        ZeroMotorsWaitCommand firstWait = new ZeroMotorsWaitCommand(3);
        ZeroMotorsWaitCommand secondWait = new ZeroMotorsWaitCommand(.5);
        ParallelCommandGroup secondTrajectNewMotor = new ParallelCommandGroup(new moveNewMotor(new NewMotor()), secondHalfTraject);
        // ProxyScheduleCommand proxy = new ProxyScheduleCommand(c_ExecUltrasonic);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(firstHalfTrajectory.getInitialPose())),
            // firstHalfTraject,
            // secondWait,
            // ultrasonicValue,
            secondTrajectNewMotor,
            firstWait
            // secondHalfTraject,
            // ultrasonicValue,
            // secondWait
        );
    }
}
