package frc.robot.subsystems.shooter.wheels;

import frc.library.StartupNeutralMode;

public interface ShooterWheelsHalI extends StartupNeutralMode {
    void coast();

    @Override
    default void setIntendedNeutralMode() {
        coast();
    }

    void setSpeed(double speedRpm);
    void setVoltage(double volts);
    double getTopSpeed();
    double getBottomSpeed();
}
