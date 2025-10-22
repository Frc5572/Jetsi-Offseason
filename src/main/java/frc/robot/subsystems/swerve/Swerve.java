package frc.robot.subsystems.swerve;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.stream.IntStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.SwerveModule;
import frc.robot.util.DrivetrainState;
import frc.robot.util.DrivetrainState.SwerveModuleStates;
import frc.robot.util.ExternalDrivetrainStateSolver;

/**
 * Swerve Subsystem
 */
public class Swerve extends SubsystemBase {

    public final DrivetrainState state;
    private final GyroIO gyro;
    private final GyroInputsAutoLogged inputs = new GyroInputsAutoLogged();
    private final SwerveModule[] modules;
    private final SwerveModuleStates moduleStates = new SwerveModuleStates();

    private final Lock odometryLock = new ReentrantLock();
    private final PhoenixOdometryThread odometryThread;

    private final Queue<Double> timestampQueue;

    public Swerve(GyroIO gyro, BiFunction<Integer, PhoenixOdometryThread, ModuleIO> moduleFn,
        Pose2d initPose, ExternalDrivetrainStateSolver... externalSolvers) {
        this.odometryThread = new PhoenixOdometryThread(odometryLock);
        this.gyro = gyro;
        this.modules = IntStream.range(0, 4)
            .mapToObj((i) -> new SwerveModule(moduleFn.apply(i, odometryThread)))
            .toArray(SwerveModule[]::new);
        odometryLock.lock();
        this.gyro.updateInputs(this.inputs);
        for (int i = 0; i < 4; i++) {
            this.modules[i].updateInputs();
        }
        odometryLock.unlock();
        for (int i = 0; i < 4; i++) {
            this.modules[i].populateStates(moduleStates, i);
        }
        this.state = new DrivetrainState(Constants.Swerve.SWERVE_KINEMATICS, moduleStates,
            Units.radiansToRotations(inputs.yawRads), initPose, externalSolvers);
        this.timestampQueue = odometryThread.makeTimestampQueue();
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        double[] timestamps = updateTimestamps();
        gyro.updateInputs(inputs);
        for (int i = 0; i < 4; i++) {
            modules[i].updateInputs();
        }
        odometryLock.unlock();

    }

    private double[] times = new double[4];
    private int numTimes = 0;

    private double[] updateTimestamps() {
        Double val;
        numTimes = 0;
        while ((val = timestampQueue.poll()) != null) {
            times[numTimes] = val;
            numTimes++;
            if (numTimes >= times.length) {
                double[] newArr = new double[times.length + 2];
                System.arraycopy(times, 0, newArr, 0, times.length);
                times = newArr;
            }
        }
        return times;
    }

}
