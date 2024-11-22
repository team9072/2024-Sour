/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import java.util.Optional;
import org.growingstems.math.RangeU;
import org.growingstems.measurements.Measurements.AngularAcceleration;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.DivByTime;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.MulByTime;
import org.growingstems.measurements.Measurements.Temperature;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Measurements.Velocity;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.measurements.Unit;

public class TalonFxActuator<P extends Unit<P> & DivByTime<V>, V extends Unit<V> & MulByTime<P>>
        extends BaseVelocityActuator<P, V> {
    public enum PositionMode {
        POSITION,
        MOTION_MAGIC;
    }

    protected final TalonFX m_motor;
    protected final TalonFXConfigurator m_configurator;

    protected V m_currentAcceleration = null;
    protected V m_currentMaxVelocity = null;

    protected PositionMode m_positionMode;
    protected int m_currentSlot = 0;
    protected Pidf m_positionPid = new Pidf();
    protected Pidf m_velocityPid = new Pidf();
    protected final Voltage m_voltageCompensation;

    public TalonFxActuator(
            TalonFX motor,
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
                BaseMotorActuator.k_defaultPowerRange);

        m_motor = motor;
        m_configurator = m_motor.getConfigurator();
        m_voltageCompensation = voltageCompensation;
    }

    public TalonFxActuator(
            TalonFX motor,
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

    public static <P extends Unit<P> & DivByTime<V>, V extends Unit<V> & MulByTime<P>>
            PositionActuator<P> positionActuator(
                    TalonFX motor,
                    P unitPerSensorUnit,
                    P startingPositionOffset,
                    RangeU<P> positionRange,
                    Voltage voltageCompensation) {
        var vNonZeroValue = unitPerSensorUnit.div(Time.seconds(1.0));
        var vZero = vNonZeroValue.sub(vNonZeroValue);

        var velocityRange = new RangeU<>(vZero, vZero);

        return new TalonFxActuator<>(
                motor,
                unitPerSensorUnit,
                startingPositionOffset,
                positionRange,
                velocityRange,
                voltageCompensation);
    }

    public static MotorActuator motorActuator(TalonFX motor, Voltage voltageCompensation) {
        var pZero = Length.ZERO;
        var positionRange = new RangeU<>(pZero, pZero);
        var velocityRange = new RangeU<>(Velocity.ZERO, Velocity.ZERO);

        return new TalonFxActuator<>(
                motor, pZero, pZero, positionRange, velocityRange, voltageCompensation);
    }

    public TalonFX getMotor() {
        return m_motor;
    }

    public void setClosedLoopSlot(int slot) {
        m_currentSlot = slot;
    }

    @Override
    public void stop() {
        m_motor.setControl(new NeutralOut());
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups) {
        if (m_disabled) {
            stop();
            return true;
        }

        // TODO: consider using acceleration term
        var control = new MotionMagicVelocityVoltage(
                velocity_sups, 0.0, true, 0.0, m_currentSlot, false, false, false);
        m_motor.setControl(control);
        return true;
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups, double arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        var control = new MotionMagicVelocityVoltage(
                velocity_sups, 0.0, true, arbitraryFeedForward, m_currentSlot, false, false, false);
        m_motor.setControl(control);
        return true;
    }

    @Override
    protected boolean setHalVelocityClosedLoop(double velocity_sups, Voltage arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        var control = new MotionMagicVelocityVoltage(
                velocity_sups,
                0.0,
                true,
                arbitraryFeedForward.asVolts(),
                m_currentSlot,
                false,
                false,
                false);
        m_motor.setControl(control);
        return true;
    }

    @Override
    protected double getHalVelocity_sups() {
        return m_motor.getVelocity().getValue();
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
    public void setPositionMode(PositionMode mode) {
        m_positionMode = mode;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su) {
        if (m_disabled) {
            stop();
            return true;
        }

        // TODO: consider using velocity term
        var control =
                new MotionMagicVoltage(position_su, true, 0.0, m_currentSlot, false, false, false);
        m_motor.setControl(control);
        return true;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su, double arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        var control = new MotionMagicVoltage(
                position_su, true, arbitraryFeedForward, m_currentSlot, false, false, false);
        m_motor.setControl(control);
        return true;
    }

    @Override
    protected boolean setHalPositionClosedLoop(double position_su, Voltage arbitraryFeedForward) {
        if (m_disabled) {
            stop();
            return true;
        }

        var control =
                new MotionMagicVoltage(position_su, true, 0.0, m_currentSlot, false, false, false);
        m_motor.setControl(control);
        return true;
    }

    public boolean setPositionClosedLoop(
            P position,
            Voltage feedforward,
            AngularVelocity velocitySu,
            AngularAcceleration accelerationSu,
            double jerkSu) {
        if (m_disabled) {
            stop();
            return false;
        }

        m_goalPosition = m_positionalRange.coerceValue(position);

        var control = new DynamicMotionMagicVoltage(
                positionToSensorUnits(m_goalPosition),
                velocitySu.asRevolutionsPerSecond(),
                accelerationSu.asRevolutionsPerSecondSquared(),
                jerkSu,
                true,
                0.0,
                m_currentSlot,
                false,
                false,
                false);
        m_motor.setControl(control);
        return true;
    }

    @Override
    protected double getHalPosition_su() {
        return m_motor.getPosition().getValue();
    }

    @Override
    protected void setHalOpenLoop(double power) {
        if (m_disabled) {
            stop();
            return;
        }

        m_motor.setControl(
                new VoltageOut(m_voltageCompensation.mul(power).asVolts(), true, false, false, false));
    }

    public void setOpenLoop(Voltage power) {
        if (m_disabled || power.abs().lt(m_voltageCompensation.mul(m_neutralDeadband))) {
            stop();
            return;
        }

        // This is honestly not used anywhere, but I'll do it to be somewhat thorough
        m_currentPower = m_powerRange.coerceValue(power.div(m_voltageCompensation).asNone());
        m_currentPower = power.div(m_voltageCompensation).asNone();

        m_motor.setControl(new VoltageOut(
                m_voltageCompensation.mul(m_currentPower).asVolts(), true, false, false, false));
    }

    @Override
    public Optional<Temperature> getTemperature() {
        return Optional.of(Temperature.celsius(m_motor.getDeviceTemp().getValue()));
    }

    @Override
    public Voltage getSupplyVoltage() {
        return Voltage.volts(m_motor.getSupplyVoltage().getValue());
    }

    @Override
    public Voltage getOutputVoltage() {
        return Voltage.volts(m_motor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    protected double getHalPositionError_su() {
        return m_motor.getClosedLoopError().getValue();
    }

    @Override
    public void setForwardSoftLimitThreshold_su(double softLimit_su) {
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        m_configurator.refresh(config);
        config.ForwardSoftLimitThreshold = softLimit_su;
        m_configurator.apply(config);
    }

    @Override
    public void setReverseSoftLimitThreshold_su(double softLimit_su) {
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        m_configurator.refresh(config);
        config.ReverseSoftLimitThreshold = softLimit_su;
        m_configurator.apply(config);
    }

    @Override
    public void enableForwardSoftLimit(boolean enable) {
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        m_configurator.refresh(config);
        config.ForwardSoftLimitEnable = enable;
        m_configurator.apply(config);
    }

    @Override
    public void enableReverseSoftLimit(boolean enable) {
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        m_configurator.refresh(config);
        config.ReverseSoftLimitEnable = enable;
        m_configurator.apply(config);
    }

    public void setAcceleration(V acceleration) {
        if (m_currentAcceleration.eq(acceleration)) {
            return;
        }

        MotionMagicConfigs config = new MotionMagicConfigs();
        m_configurator.refresh(config);
        config.MotionMagicAcceleration =
                acceleration.div(m_velocityUnitPerSensorUnit).asNone();
        m_configurator.apply(config);
    }

    @Override
    public void setMaxVelocity(V maxVelocity) {
        if (m_currentMaxVelocity != null && m_currentMaxVelocity.eq(maxVelocity)) {
            return;
        }

        MotionMagicConfigs config = new MotionMagicConfigs();
        m_configurator.refresh(config);
        config.MotionMagicCruiseVelocity =
                maxVelocity.div(m_velocityUnitPerSensorUnit).asNone();
        m_configurator.apply(config);
    }

    @Override
    public boolean enableCurrentLimiting(boolean enable) {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        m_configurator.refresh(config);
        config.SupplyCurrentLimitEnable = enable;
        m_configurator.apply(config);

        return true;
    }

    @Override
    public Optional<Current> getStatorCurrent() {
        return Optional.of(Current.amps(m_motor.getTorqueCurrent().getValue()));
    }

    @Override
    public Optional<Current> getSupplyCurrent() {
        return Optional.of(Current.amps(m_motor.getSupplyCurrent().getValue()));
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return m_motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    @Override
    public boolean getReverseLimitSwitch() {
        return m_motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    @Override
    public void setCoast() {
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();
        m_configurator.refresh(motorOutputConfig);
        motorOutputConfig.NeutralMode = NeutralModeValue.Coast;
        m_configurator.apply(motorOutputConfig);
    }

    @Override
    public void setBrake() {
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();
        m_configurator.refresh(motorOutputConfig);
        motorOutputConfig.NeutralMode = NeutralModeValue.Brake;
        m_configurator.apply(motorOutputConfig);
    }
}
