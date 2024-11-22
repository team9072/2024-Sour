/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.Optional;
import org.growingstems.math.RangeU;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.DivByTime;
import org.growingstems.measurements.Measurements.MulByTime;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.measurements.Unit;

public class TalonSrxActuator<P extends Unit<P> & DivByTime<V>, V extends Unit<V> & MulByTime<P>>
        extends CtreBaseMotorControllerActuator<P, V> {
    protected final TalonSRX m_motor;

    public TalonSrxActuator(
            TalonSRX motor,
            P unitPerSensorUnit,
            P startingPositionOffset,
            RangeU<P> positionRange,
            RangeU<V> velocityRange,
            PositionMode positionMode,
            Voltage voltageCompensation) {
        super(
                motor,
                unitPerSensorUnit,
                startingPositionOffset,
                positionRange,
                velocityRange,
                positionMode,
                voltageCompensation);

        m_motor = motor;
    }

    public TalonSrxActuator(
            TalonSRX motor,
            P unitPerSensorUnit,
            P startingPositionOffset,
            RangeU<P> positionRange,
            RangeU<V> velocityRange,
            Voltage voltageCompensation) {
        this(
                motor,
                unitPerSensorUnit,
                startingPositionOffset,
                positionRange,
                velocityRange,
                PositionMode.POSITION,
                voltageCompensation);
    }

    public TalonSRX getMotor() {
        return m_motor;
    }

    @Override
    public boolean enableCurrentLimiting(boolean enable) {
        m_motor.enableCurrentLimit(enable);
        return true;
    }

    @Override
    public Optional<Current> getStatorCurrent() {
        return Optional.empty();
    }

    @Override
    public Optional<Current> getSupplyCurrent() {
        return Optional.of(Current.amps(m_motor.getSupplyCurrent()));
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return m_motor.isFwdLimitSwitchClosed() == 1;
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return m_motor.isRevLimitSwitchClosed() == 1;
    }
}
