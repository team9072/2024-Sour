/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import org.growingstems.math.Range;
import org.growingstems.math.RangeU;
import org.growingstems.measurements.Measurements.DivByTime;
import org.growingstems.measurements.Measurements.MulByTime;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.measurements.Unit;

public abstract class BaseVelocityActuator<
                P extends Unit<P> & DivByTime<V>, V extends Unit<V> & MulByTime<P>>
        extends BasePositionActuator<P> implements VelocityActuator<P, V> {
    protected final RangeU<V> m_velocityRange;
    protected final V m_velocityUnitPerSensorUnit;

    protected V m_goalVelocity;

    protected BaseVelocityActuator(
            P unitPerSensorUnit,
            V velocityUnitPerSensorUnit,
            P startingPositionOffset,
            RangeU<P> positionRange,
            RangeU<V> velocityRange) {
        this(
                unitPerSensorUnit,
                velocityUnitPerSensorUnit,
                startingPositionOffset,
                positionRange,
                velocityRange,
                k_defaultPowerRange);
    }

    protected BaseVelocityActuator(
            P unitPerSensorUnit,
            V velocityUnitPerSensorUnit,
            P startingPositionOffset,
            RangeU<P> positionRange,
            RangeU<V> velocityRange,
            Range powerRange) {
        super(unitPerSensorUnit, startingPositionOffset, positionRange, powerRange);
        m_velocityUnitPerSensorUnit = velocityUnitPerSensorUnit;
        m_velocityRange = velocityRange;
        m_goalVelocity = velocityUnitPerSensorUnit.mul(Double.NaN);
    }

    /**
     * Puts the drive wheel of the module into velocity closed loop control and sets the module's
     * drive wheel velocity goal.
     *
     * @param velocity_sups Velocity goal to be used for velocity closed loop in sensor units per
     *     second. (-inf, inf)
     * @return True if the extending class implements this function.
     */
    protected abstract boolean setHalVelocityClosedLoop(double velocity_sups);

    protected abstract boolean setHalVelocityClosedLoop(
            double velocity_sups, double arbitraryFeedForward);

    protected abstract boolean setHalVelocityClosedLoop(
            double velocity_sups, Voltage arbitraryFeedForward);

    @Override
    public RangeU<V> getVelocityRange() {
        return m_velocityRange;
    }

    @Override
    public boolean setVelocityClosedLoop(V velocity) {
        if (m_disabled) {
            stop();
            return false;
        }

        m_goalVelocity = m_velocityRange.coerceValue(velocity);
        return setHalVelocityClosedLoop(velocityToSensorUnits(m_goalVelocity));
    }

    @Override
    public boolean setVelocityClosedLoop(V velocity, double feedforward) {
        if (m_disabled) {
            stop();
            return false;
        }

        m_goalVelocity = m_velocityRange.coerceValue(velocity);
        return setHalVelocityClosedLoop(velocityToSensorUnits(m_goalVelocity), feedforward);
    }

    protected double velocityToSensorUnits(V velocity) {
        return velocity.div(m_velocityUnitPerSensorUnit).asNone();
    }

    @Override
    public V getVelocity() {
        return m_velocityUnitPerSensorUnit.mul(getHalVelocity_sups());
    }

    @Override
    public V getGoalVelocity() {
        return m_goalVelocity;
    }

    @Override
    public V getVelocityError() {
        return getGoalVelocity().sub(getVelocity());
    }

    /**
     * Gets the swerve module's current drive wheel velocity directly from the module's sensor.
     * Generally makes a direct call to the hardware to retrieve sensor data.
     *
     * @return drive wheel velocity in raw sensor units per second. (-inf, inf)
     */
    protected abstract double getHalVelocity_sups();
}
