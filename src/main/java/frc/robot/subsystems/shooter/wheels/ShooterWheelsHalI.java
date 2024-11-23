package frc.robot.subsystems.shooter.wheels;

import frc.library.StartupNeutralMode;

public interface ShooterWheelsHalI extends StartupNeutralMode {
    void brake();

    void coast();

    @Override
    default void setIntendedNeutralMode() {
        coast();
    }

    /**
     * Run SysId for one motor
     *
     * @param volts voltage to run motor at
     * @param top Whether to run top or bottom motor
     */
    void setVoltageSysId(double volts, boolean top);

    void setSpeed(double speedRpm);

    double getTopSpeed();

    double getBottomSpeed();
}
