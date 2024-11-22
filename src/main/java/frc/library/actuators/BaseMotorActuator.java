/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import org.growingstems.math.Range;

public abstract class BaseMotorActuator implements MotorActuator {
    protected static final Range k_defaultPowerRange = new Range(-1.0, 1.0);
    protected final Range m_powerRange;
    protected boolean m_disabled = false;

    protected double m_currentPower = 0.0;
    protected double m_neutralDeadband = 1.0e-9;

    protected BaseMotorActuator() {
        this(k_defaultPowerRange);
    }

    protected BaseMotorActuator(Range powerRange) {
        m_powerRange = powerRange;
    }

    @Override
    public void setNeutralDeadband(double neutralDeadband) {
        m_neutralDeadband = neutralDeadband;
    }

    @Override
    public double getNeutralDeadband() {
        return m_neutralDeadband;
    }

    @Override
    public void setOpenLoop(double power) {
        if (m_disabled) {
            stop();
            return;
        }

        if (Math.abs(power) < m_neutralDeadband) {
            stop();
            return;
        }

        m_currentPower = m_powerRange.coerceValue(power);
        setHalOpenLoop(m_currentPower);
    }

    @Override
    public double getCurrentPower() {
        return m_currentPower;
    }

    @Override
    public void disable() {
        stop();
        m_disabled = true;
    }

    @Override
    public void enable() {
        m_disabled = false;
    }

    @Override
    public boolean isDisabled() {
        return m_disabled;
    }

    /**
     * Puts the drive wheel controller of the drive wheel into open loop control.
     *
     * @param power The power setting for the drive wheel controller. [-1.0, 1.0]
     */
    protected abstract void setHalOpenLoop(double power);
}
