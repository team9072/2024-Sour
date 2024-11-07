package frc.robot.subsystems.intake.rollers;

import frc.library.StartupNeutralMode;
import org.growingstems.measurements.Measurements.Voltage;

public interface IntakeRollersHalI extends StartupNeutralMode {
    void brake();

    void coast();

    void setPower(Voltage power);

    void update();
}
