package frc.robot.subsystems.elevator.rollers;

import org.growingstems.measurements.Measurements.Voltage;

public class ElevatorRollersDummyHal implements ElevatorRollersHalI {
    @Override
    public void setPower(Voltage power) {
        // NOP
    }

    @Override
    public void setIntendedNeutralMode() {
        // NOP
    }

    @Override
    public void update() {
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
