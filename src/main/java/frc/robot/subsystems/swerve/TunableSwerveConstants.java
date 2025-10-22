package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.Consumer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import lombok.Builder;

@Builder
public class TunableSwerveConstants implements Sendable {

    public Distance wheelRadius;
    public LinearVelocity maxSpeed;
    public AngularVelocity maxAngularVelocity;
    public double drivekS;
    public double drivekV;
    public double drivekA;
    public double drivekP;
    public double drivekD;
    public double anglekP;
    public double anglekD;

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("wheelRadius (in)", () -> this.getWheelRadius().in(Inches),
            (v) -> this.setWheelRadius(Inches.of(v)));
        builder.addDoubleProperty("maxSpeed", () -> this.getMaxSpeed().in(MetersPerSecond),
            (v) -> this.setMaxSpeed(MetersPerSecond.of(v)));
        builder.addDoubleProperty("maxAngularVelocity",
            () -> this.getMaxAngularVelocity().in(RotationsPerSecond),
            (v) -> this.setMaxAngularVelocity(RotationsPerSecond.of(v)));
        builder.addDoubleProperty("drivekS", this::getDrivekS, this::setDrivekS);
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
    boolean dirty = false;

    public Distance getWheelRadius() {
        return wheelRadius;
    }

    public void setWheelRadius(Distance wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    public LinearVelocity getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(LinearVelocity maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public AngularVelocity getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public void setMaxAngularVelocity(AngularVelocity maxAngularVelocity) {
        this.maxAngularVelocity = maxAngularVelocity;
    }

    public double getDrivekS() {
        return drivekS;
    }

    public void setDrivekS(double drivekS) {
        this.drivekS = drivekS;
    }

    public double getDrivekV() {
        return drivekV;
    }

    public void setDrivekV(double drivekV) {
        this.drivekV = drivekV;
    }

    public double getDrivekA() {
        return drivekA;
    }

    public void setDrivekA(double drivekA) {
        this.drivekA = drivekA;
    }

    public double getDrivekP() {
        return drivekP;
    }

    public void setDrivekP(double drivekP) {
        this.drivekP = drivekP;
    }

    public double getDrivekD() {
        return drivekD;
    }

    public void setDrivekD(double drivekD) {
        this.drivekD = drivekD;
    }

    public double getAnglekP() {
        return anglekP;
    }

    public void setAnglekP(double anglekP) {
        this.anglekP = anglekP;
    }

    public double getAnglekD() {
        return anglekD;
    }

    public void setAnglekD(double anglekD) {
        this.anglekD = anglekD;
    }

    public boolean isDirty() {
        return dirty;
    }

    public void setDirty(boolean dirty) {
        this.dirty = dirty;
    }



}
