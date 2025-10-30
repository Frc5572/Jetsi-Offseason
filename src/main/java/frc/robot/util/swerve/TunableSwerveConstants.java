package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.Consumer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import lombok.Builder;

@Builder
public class TunableSwerveConstants implements Sendable {

    private Distance wheelRadius;
    private LinearVelocity maxSpeed;
    private LinearAcceleration maxAcceleration;
    private AngularVelocity maxAngularVelocity;
    private AngularVelocity maxSterringVelocity;
    private double drivekS;
    private double drivekV;
    private double drivekA;
    private double drivekT;
    private double drivekP;
    private double drivekD;
    private double anglekP;
    private double anglekD;

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("wheelRadius", () -> this.getWheelRadius().in(Inches),
            (v) -> this.setWheelRadius(Inches.of(v)));
        builder.addDoubleProperty("maxSpeed", () -> this.getMaxSpeed().in(MetersPerSecond),
            (v) -> this.setMaxSpeed(MetersPerSecond.of(v)));
        builder.addDoubleProperty("maxAccel",
            () -> this.getMaxAcceleration().in(MetersPerSecondPerSecond),
            (v) -> this.setMaxAcceleration(MetersPerSecondPerSecond.of(v)));
        builder.addDoubleProperty("maxSteeringVelocity",
            () -> this.getMaxSterringVelocity().in(RotationsPerSecond),
            (v) -> this.setMaxSterringVelocity(RotationsPerSecond.of(v)));
        builder.addDoubleProperty("maxAngularVelocity",
            () -> this.getMaxAngularVelocity().in(RotationsPerSecond),
            (v) -> this.setMaxAngularVelocity(RotationsPerSecond.of(v)));
        builder.addDoubleProperty("drivekS", this::getDrivekS, this::setDrivekS);
        builder.addDoubleProperty("drivekT", this::getDrivekT, this::setDrivekT);
        builder.addDoubleProperty("drivekV", this::getDrivekV, this::setDrivekV);
        builder.addDoubleProperty("drivekA", this::getDrivekA, this::setDrivekA);
        builder.addDoubleProperty("drivekP", this::getDrivekP, this::setDrivekP);
        builder.addDoubleProperty("drivekD", this::getDrivekD, this::setDrivekD);
        builder.addDoubleProperty("anglekP", this::getAnglekP, this::setAnglekP);
        builder.addDoubleProperty("anglekD", this::getAnglekD, this::setAnglekD);
    }

    public void maybeUpdate(Consumer<TunableSwerveConstants> updateFun) {
        if (dirty) {
            updateFun.accept(this);
            dirty = false;
        }
    }

    @Builder.Default
    boolean drivePIDdirty = false;
    @Builder.Default
    boolean anglePIDdirty = false;
    @Builder.Default
    boolean dirty = false;

    public Distance getWheelRadius() {
        return wheelRadius;
    }

    public void setWheelRadius(Distance wheelRadius) {
        this.dirty = true;
        this.wheelRadius = wheelRadius;
    }

    public LinearVelocity getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(LinearVelocity maxSpeed) {
        this.dirty = true;
        this.maxSpeed = maxSpeed;
    }

    public AngularVelocity getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public void setMaxAngularVelocity(AngularVelocity maxAngularVelocity) {
        this.dirty = true;
        this.maxAngularVelocity = maxAngularVelocity;
    }

    public double getDrivekS() {
        return drivekS;
    }

    public void setDrivekS(double drivekS) {
        this.drivePIDdirty = true;
        this.dirty = true;
        this.drivekS = drivekS;
    }

    public double getDrivekT() {
        this.dirty = true;
        return drivekT;
    }

    public void setDrivekT(double drivekT) {
        this.dirty = true;
        this.drivekT = drivekT;
    }

    public double getDrivekV() {
        return drivekV;
    }

    public void setDrivekV(double drivekV) {
        this.drivePIDdirty = true;
        this.dirty = true;
        this.drivekV = drivekV;
    }

    public double getDrivekA() {
        return drivekA;
    }

    public void setDrivekA(double drivekA) {
        this.drivePIDdirty = true;
        this.dirty = true;
        this.drivekA = drivekA;
    }

    public double getDrivekP() {
        return drivekP;
    }

    public void setDrivekP(double drivekP) {
        this.drivePIDdirty = true;
        this.dirty = true;
        this.drivekP = drivekP;
    }

    public double getDrivekD() {
        return drivekD;
    }

    public void setDrivekD(double drivekD) {
        this.drivePIDdirty = true;
        this.dirty = true;
        this.drivekD = drivekD;
    }

    public double getAnglekP() {
        return anglekP;
    }

    public void setAnglekP(double anglekP) {
        this.anglePIDdirty = true;
        this.dirty = true;
        this.anglekP = anglekP;
    }

    public double getAnglekD() {
        return anglekD;
    }

    public void setAnglekD(double anglekD) {
        this.anglePIDdirty = true;
        this.dirty = true;
        this.anglekD = anglekD;
    }

    public LinearAcceleration getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(LinearAcceleration maxAcceleration) {
        dirty = true;
        this.maxAcceleration = maxAcceleration;
    }

    public AngularVelocity getMaxSterringVelocity() {
        return maxSterringVelocity;
    }

    public void setMaxSterringVelocity(AngularVelocity maxSterringVelocity) {
        dirty = true;
        this.maxSterringVelocity = maxSterringVelocity;
    }

    public boolean isDirty() {
        return dirty;
    }

    public void clean() {
        this.dirty = false;
        this.anglePIDdirty = false;
        this.drivePIDdirty = false;
    }

    public boolean isDrivePIDdirty() {
        return drivePIDdirty;
    }

    public boolean isAnglePIDdirty() {
        return anglePIDdirty;
    }

    public void publish() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("/Tuning");
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(table);
        SendableRegistry.publish(this, builder);
        builder.startListeners();
        table.getEntry(".name").setString("Swerve Tunables");
    }

}
