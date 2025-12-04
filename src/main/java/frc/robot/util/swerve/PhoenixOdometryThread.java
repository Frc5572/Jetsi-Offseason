package frc.robot.util.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class PhoenixOdometryThread extends Thread {

    private final Lock odometryLock;
    private final Lock signalsLock = new ReentrantLock();

    private BaseStatusSignal[] phoenixSignals;
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private final boolean isCANFD = new CANBus("*").isNetworkFD();

    public PhoenixOdometryThread(Lock odometryLock) {
        this.odometryLock = odometryLock;
        this.setName("PhoenixOdometryThread");
        this.setDaemon(true);
    }

    @Override
    public synchronized void start() {
        // Only start the thread if stuff is actually registered.
        if (timestampQueues.size() > 0) {
            super.start();
        }
    }

    /** Registers a Phoenix signal to be read from the thread. */
    public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            odometryLock.unlock();
        }
        return queue;
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            signalsLock.unlock();
            odometryLock.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            signalsLock.lock();
            try {
                if (isCANFD && phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(2.0 / Constants.Swerve.odometryFrequency,
                        phoenixSignals);
                } else {
                    Thread.sleep((long) (1000.0 / Constants.Swerve.odometryFrequency));
                    if (phoenixSignals.length > 0) {
                        BaseStatusSignal.refreshAll(phoenixSignals);
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            odometryLock.lock();
            try {
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }

                for (int i = 0; i < phoenixSignals.length; i++) {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            } finally {
                odometryLock.unlock();
            }
        }
    }

}
