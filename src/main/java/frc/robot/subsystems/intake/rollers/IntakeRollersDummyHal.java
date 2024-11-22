package frc.robot.subsystems.intake.rollers;

import org.growingstems.measurements.Measurements.Voltage;

public class IntakeRollersDummyHal implements IntakeRollersHalI {
    @Override
    public void setPower(Voltage power) {
        // NOP
    }

    @Override
    public void setIntendedNeutralMode() {
        // NOP
    }

    @Override
    public void brake() {
        // NOP
    }

    @Override
    public void coast() {
        // NOP
    }
}
