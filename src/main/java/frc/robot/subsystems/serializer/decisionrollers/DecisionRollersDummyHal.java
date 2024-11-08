package frc.robot.subsystems.serializer.decisionrollers;

import org.growingstems.measurements.Measurements.Voltage;

public class DecisionRollersDummyHal implements DecisionRollersHalI {
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
