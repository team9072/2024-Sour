/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import org.growingstems.math.Range;
import org.growingstems.math.RangeU;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.measurements.Unit;

public abstract class BasePositionActuator<P extends Unit<P>> extends BaseMotorActuator
        implements PositionActuator<P> {
    protected RangeU<P> m_positionalRange;
    protected final P m_unitPerSensorUnit;

    private P m_positionalOffset;

    protected P m_goalPosition;

    protected BasePositionActuator(
            P unitPerSensorUnit, P startingPositionOffset, RangeU<P> positionRange) {
        this(unitPerSensorUnit, startingPositionOffset, positionRange, k_defaultPowerRange);
    }

    protected BasePositionActuator(
            P unitPerSensorUnit, P startingPositionOffset, RangeU<P> positionRange, Range powerRange) {
        super(powerRange);
        m_unitPerSensorUnit = unitPerSensorUnit;
        m_positionalOffset = startingPositionOffset;
        m_positionalRange = positionRange;
        m_goalPosition = unitPerSensorUnit.mul(Double.NaN);
    }

    /**
     * Puts the drive wheel of the module into position closed loop control and sets the module's
     * drive wheel position goal.
     *
     * @param position_su Position goal to be used for position closed loop in sensor units. (-inf,
     *     inf)
     * @return True if the extending class implements this function.
     */
    protected abstract boolean setHalPositionClosedLoop(double position_su);

    protected abstract boolean setHalPositionClosedLoop(
            double position_su, double arbitraryFeedForward);

    protected abstract boolean setHalPositionClosedLoop(
            double position_su, Voltage arbitraryFeedForward);

    @Override
    public RangeU<P> getPositionalRange() {
        return m_positionalRange;
    }

    @Override
    public void calibrateOffset(P physicalPosition) {
        m_positionalOffset = physicalPosition.sub(getRawPosition());
    }

    @Override
    public boolean setPositionClosedLoop(P position) {
        if (m_disabled) {
            stop();
            return false;
        }

        m_goalPosition = m_positionalRange.coerceValue(position);
        return setHalPositionClosedLoop(positionToSensorUnits(m_goalPosition));
    }

    @Override
    public boolean setPositionClosedLoop(P position, double feedforward) {
        if (m_disabled) {
            stop();
            return false;
        }

        m_goalPosition = m_positionalRange.coerceValue(position);
        return setHalPositionClosedLoop(positionToSensorUnits(m_goalPosition), feedforward);
    }

    protected double positionToSensorUnits(P position) {
        return position.sub(m_positionalOffset).div(m_unitPerSensorUnit).asNone();
    }

    @Override
    public P getPosition() {
        return getRawPosition().add(m_positionalOffset);
    }

    /**
     * Gets the scaled position without applying the offset
     *
     * @return the non-offset position
     */
    public P getRawPosition() {
        return m_unitPerSensorUnit.mul(getHalPosition_su());
    }

    @Override
    public P getGoalPosition() {
        return m_goalPosition;
    }

    @Override
    public P getPositionError() {
        return getGoalPosition().sub(getPosition());
    }

    /**
     * Gets the swerve module's current drive position directly from the module's sensor. Generally
     * makes a direct call to the hardware to retrieve sensor data.
     *
     * @return drive wheel position in raw sensor units. (-inf, inf)
     */
    protected abstract double getHalPosition_su();

    protected abstract double getHalPositionError_su();

    public P getUnitPerSensorUnit() {
        return m_unitPerSensorUnit;
    }
}
