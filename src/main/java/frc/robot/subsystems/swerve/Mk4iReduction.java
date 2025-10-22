package frc.robot.subsystems.swerve;

public enum Mk4iReduction {
    L1(8.14), L2(6.75), L3(6.12), TURN((150.0 / 7.0));

    public final double reduction;

    Mk4iReduction(double reduction) {
        this.reduction = reduction;
    }

}
