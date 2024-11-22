/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import java.util.Optional;
import org.growingstems.math.RangeU;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.DivByTime;
import org.growingstems.measurements.Measurements.MulByTime;
import org.growingstems.measurements.Measurements.Temperature;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.measurements.Unit;

public class DummyActuator<P extends Unit<P> & DivByTime<V>, V extends Unit<V> & MulByTime<P>>
        extends BaseVelocityActuator<P, V> {
    private double m_currentlySetPosition_su = 0.0;
    private double m_currentlySetVelocity_sups = 0.0;

    public DummyActuator(
            P unitPerSensorUnit,
            V velocityUnitPerSensorUnit,
            P startingPositionOffset,
            RangeU<P> positionRange,
            RangeU<V> velocityRange,
            P goalPosition,
            V goalVelocity) {
        super(
                unitPerSensorUnit,
                velocityUnitPerSensorUnit,
                startingPositionOffset,
                positionRange,
                velocityRange);
        m_goalPosition = goalPosition;
        m_goalVelocity = goalVelocity;
    }

    @Override
    public void stop() {
        // NOP
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups) {
        if (m_disabled) {
            stop();
            return true;
        }

        m_currentlySetVelocity_sups = velocity_sups;
        return true;
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups, double arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        m_currentlySetVelocity_sups = velocity_sups;
        return true;
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups, Voltage arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        return true;
    }

    @Override
    protected double getHalVelocity_sups() {
        return m_currentlySetVelocity_sups;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su) {
        if (m_disabled) {
            stop();
            return true;
        }

        m_currentlySetPosition_su = position_su;
        return true;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su, double arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        m_currentlySetPosition_su = position_su;
        return true;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su, Voltage arbitraryFeedForward) {
        m_currentlySetPosition_su = position_su;
        return false;
    }

    @Override
    protected double getHalPosition_su() {
        return m_currentlySetPosition_su;
    }

    @Override
    protected double getHalPositionError_su() {
        return 0.0;
    }

    @Override
    protected void setHalOpenLoop(double power) {
        if (m_disabled) {
            stop();
            return;
        }
    }

    @Override
    public boolean enableCurrentLimiting(boolean enable) {
        return false;
    }

    @Override
    public Optional<Temperature> getTemperature() {
        return Optional.of(Temperature.ZERO);
    }

    @Override
    public Voltage getSupplyVoltage() {
        return Voltage.ZERO;
    }

    @Override
    public Voltage getOutputVoltage() {
        return Voltage.ZERO;
    }

    @Override
    public Optional<Current> getStatorCurrent() {
        return Optional.of(Current.ZERO);
    }

    @Override
    public Optional<Current> getSupplyCurrent() {
        return Optional.of(Current.ZERO);
    }

    @Override
    public void setReverseSoftLimitThreshold_su(double softLimit_su) {
        // NOP
    }

    @Override
    public void setForwardSoftLimitThreshold_su(double softLimit_su) {
        // NOP
    }

    @Override
    public void enableForwardSoftLimit(boolean enable) {
        // NOP
    }

    @Override
    public void enableReverseSoftLimit(boolean enable) {
        // NOP
    }

    @Override
    public void setMaxVelocity(V maxVelocity) {
        // NOP
    }

    @Override
    public void setAcceleration(V acceleration) {
        // NOP
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
        // NOP
    }

    @Override
    public void setBrake() {
        // NOP
    }
}
