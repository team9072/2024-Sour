/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import frc.library.drivers.ctre.BaseCanMotorControllerUtils.PidfIdx;
import frc.library.drivers.ctre.BaseCanMotorControllerUtils.PidfSlot;
import java.util.Optional;
import org.growingstems.math.Range;
import org.growingstems.math.RangeU;
import org.growingstems.measurements.Measurements.DivByTime;
import org.growingstems.measurements.Measurements.MulByTime;
import org.growingstems.measurements.Measurements.Temperature;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.measurements.Unit;

public abstract class CtreBaseMotorControllerActuator<
                P extends Unit<P> & DivByTime<V>, V extends Unit<V> & MulByTime<P>>
        extends BaseVelocityActuator<P, V> {
    protected final BaseMotorController m_motor;

    protected static final PidfIdx k_defaultPidIdx = PidfIdx.PRIMARY;
    protected static final PidfSlot k_positionPidSlot = PidfSlot.SLOT_0;
    protected static final PidfSlot k_velocityPidSlot = PidfSlot.SLOT_1;
    protected static final PidfSlot k_motionMagicPidSlot = PidfSlot.SLOT_2;

    protected V m_currentAcceleration = null;
    protected V m_currentMaxVelocity = null;

    public enum PositionMode {
        POSITION,
        MOTION_MAGIC;
    }

    public enum ClosedLoopSlot {
        POSITION(k_positionPidSlot),
        VELOCITY(k_velocityPidSlot),
        MOTION_MAGIC(k_motionMagicPidSlot);

        public final PidfSlot slot;

        private ClosedLoopSlot(PidfSlot slot) {
            this.slot = slot;
        }
    }

    protected PositionMode m_positionMode;
    protected ClosedLoopSlot m_currentSlot = null;
    protected Pidf m_positionPid = new Pidf();
    protected Pidf m_velocityPid = new Pidf();
    protected final Voltage m_voltageCompensation;

    public CtreBaseMotorControllerActuator(
            BaseMotorController motor,
            P unitPerSensorUnit,
            P startingPositionOffset,
            RangeU<P> positionRange,
            RangeU<V> velocityRange,
            PositionMode positionMode,
            Voltage voltageCompensation) {
        super(
                unitPerSensorUnit,
                unitPerSensorUnit.div(Time.seconds(1.0)),
                startingPositionOffset,
                positionRange,
                velocityRange,
                new Range(-1.0, 1.0));

        m_positionMode = positionMode;
        m_motor = motor;
        m_voltageCompensation = voltageCompensation;
    }

    @Override
    public void stop() {
        m_motor.set(ControlMode.Disabled, 0.0);
    }

    protected void setPidSlot(ClosedLoopSlot slot) {
        if (m_currentSlot != slot) {
            // It was slot.slot.slot originally
            m_motor.selectProfileSlot(slot.slot.num, k_defaultPidIdx.idx);
            m_currentSlot = slot;
        }
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups) {
        if (m_disabled) {
            stop();
            return true;
        }

        setPidSlot(ClosedLoopSlot.VELOCITY);
        m_motor.set(ControlMode.Velocity, velocity_sups);
        return true;
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups, double arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        setPidSlot(ClosedLoopSlot.VELOCITY);
        m_motor.set(
                ControlMode.Velocity, velocity_sups, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
        return true;
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups, Voltage arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        setPidSlot(ClosedLoopSlot.VELOCITY);
        m_motor.set(
                ControlMode.Velocity,
                velocity_sups,
                DemandType.ArbitraryFeedForward,
                arbitraryFeedForward.div(m_voltageCompensation).asNone());
        return true;
    }

    @Override
    protected double getHalVelocity_sups() {
        return m_motor.getSelectedSensorVelocity() * 10.0;
    }

    /**
     * Sets the position closed loop mode for the given CTRE based actuator. Does not directly effect
     * the controller's closed loop strategy until the next time the controller's position is set.
     *
     * <p>This setting effects both the set position functions, and the the set position PIDF
     * functions.
     *
     * @param mode the position closed loop mode to use.
     */
    protected void setPositionMode(PositionMode mode) {
        m_positionMode = mode;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su) {
        if (m_disabled) {
            stop();
            return true;
        }

        /*
         * Let it be known that just before midnight, on 1/28/23, Nick Aucoin wasted
         * ~6 hours on an issue that was solved by changing the word 'Position' below
         * to 'MotionMagic' fixed a bug that COULD NOT be solved. Absolutely nothing
         * made sense and I was actually losing my mind before I came to the solution
         * that finally got the Swerve Module Refactor code working.
         *
         * UPDATE: On 2/7/23, we spent another 1.5 hours because of the same issue.
         */
        switch (m_positionMode) {
            case POSITION:
                setPidSlot(ClosedLoopSlot.POSITION);
                m_motor.set(ControlMode.Position, position_su);
                return true;
            case MOTION_MAGIC:
                setPidSlot(ClosedLoopSlot.MOTION_MAGIC);
                m_motor.set(ControlMode.MotionMagic, position_su);
                return true;
        }

        return false;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su, double arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        switch (m_positionMode) {
            case POSITION:
                setPidSlot(ClosedLoopSlot.POSITION);
                m_motor.set(
                        ControlMode.Position,
                        position_su,
                        DemandType.ArbitraryFeedForward,
                        arbitraryFeedForward);
                return true;
            case MOTION_MAGIC:
                setPidSlot(ClosedLoopSlot.MOTION_MAGIC);
                m_motor.set(
                        ControlMode.MotionMagic,
                        position_su,
                        DemandType.ArbitraryFeedForward,
                        arbitraryFeedForward);
                return true;
        }

        return false;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su, Voltage arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        return setHalPositionClosedLoop(
                position_su, arbitraryFeedForward.div(m_voltageCompensation).asNone());
    }

    @Override
    protected double getHalPosition_su() {
        return m_motor.getSelectedSensorPosition();
    }

    @Override
    protected void setHalOpenLoop(double power) {
        if (m_disabled) {
            stop();
            return;
        }

        m_motor.set(ControlMode.PercentOutput, power);
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
    protected double getHalPositionError_su() {
        return m_motor.getClosedLoopError();
    }

    @Override
    public void setForwardSoftLimitThreshold_su(double softLimit_su) {
        m_motor.configForwardSoftLimitThreshold(softLimit_su);
    }

    @Override
    public void setReverseSoftLimitThreshold_su(double softLimit_su) {
        m_motor.configReverseSoftLimitThreshold(softLimit_su);
    }

    @Override
    public void enableForwardSoftLimit(boolean enable) {
        m_motor.configForwardSoftLimitEnable(enable);
    }

    @Override
    public void enableReverseSoftLimit(boolean enable) {
        m_motor.configReverseSoftLimitEnable(enable);
    }

    public void setAcceleration(V acceleration) {
        if (m_currentAcceleration != null && m_currentAcceleration.eq(acceleration)) {
            return;
        }
        m_motor.configMotionAcceleration(
                acceleration.div(m_velocityUnitPerSensorUnit).asNone() / 10.0);
        m_currentAcceleration = acceleration;
    }

    @Override
    public void setMaxVelocity(V maxVelocity) {
        if (m_currentMaxVelocity != null && m_currentMaxVelocity.eq(maxVelocity)) {
            return;
        }
        m_motor.configMotionCruiseVelocity(
                maxVelocity.div(m_velocityUnitPerSensorUnit).asNone() / 10.0);
        m_currentMaxVelocity = maxVelocity;
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
