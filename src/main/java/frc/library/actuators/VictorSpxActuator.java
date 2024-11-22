/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import java.util.Optional;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.Temperature;
import org.growingstems.measurements.Measurements.Voltage;

public class VictorSpxActuator extends BaseMotorActuator {
    protected final VictorSPX m_motor;

    public VictorSpxActuator(VictorSPX motor) {
        m_motor = motor;
    }

    public VictorSPX getMotor() {
        return m_motor;
    }

    @Override
    public void stop() {
        m_motor.set(VictorSPXControlMode.Disabled, 0.0);
    }

    @Override
    protected void setHalOpenLoop(double power) {
        if (m_disabled) {
            stop();
            return;
        }

        m_motor.set(VictorSPXControlMode.PercentOutput, power);
    }

    @Override
    public boolean enableCurrentLimiting(boolean enable) {
        return false;
    }

    @Override
    public Optional<Temperature> getTemperature() {
        return Optional.of(Temperature.celsius(m_motor.getTemperature()));
    }

    @Override
    public Voltage getSupplyVoltage() {
        return Voltage.volts(m_motor.getBusVoltage());
    }

    @Override
    public Voltage getOutputVoltage() {
        return Voltage.volts(m_motor.getMotorOutputVoltage());
    }

    @Override
    public Optional<Current> getStatorCurrent() {
        return Optional.empty();
    }

    @Override
    public Optional<Current> getSupplyCurrent() {
        return Optional.empty();
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return false;
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return false;
    }

    @Override
    public void setCoast() {
        m_motor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void setBrake() {
        m_motor.setNeutralMode(NeutralMode.Brake);
    }
}
