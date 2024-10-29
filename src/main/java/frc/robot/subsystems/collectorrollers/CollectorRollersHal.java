/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.collectorrollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.library.CtreUtils;
import frc.robot.Robot;
import org.growingstems.control.actuators.MotorActuator;
import org.growingstems.frc.actuators.TalonFxActuator;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.Voltage;

public class CollectorRollersHal implements CollectorRollersHalI {

    // Motor
    private final MotorActuator m_actuator;
    private final TalonFX m_motor;
    private final TalonFXConfigurator m_configurator;

    // Motor Configuration
    private static final int k_canId = 39;
    private static final NeutralModeValue k_intendedNeutralMode = NeutralModeValue.Brake;

    // Logging
    // private final Consumer<Voltage> m_logPower;
    // private final Consumer<Current> m_logSupplyCurrent;
    // private final Consumer<Current> m_logStatorCurrent;

    // public GrowlerCollectorRollersHal(LogBuilder builder) {
    public CollectorRollersHal() {
        // --------------------
        //    Motor Settings
        // --------------------
        m_motor = new TalonFX(k_canId, Robot.k_canivoreCan);
        m_configurator = m_motor.getConfigurator();

        // Reset to Factory Defaults
        var configuration = new TalonFXConfiguration();

        // General Motor Settings
        var general = new MotorOutputConfigs();
        general.NeutralMode = k_intendedNeutralMode;
        general.Inverted = InvertedValue.Clockwise_Positive;
        configuration.withMotorOutput(general);

        // Current Limiting
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = Current.amps(25.0).asAmps();
        configuration.withCurrentLimits(currentLimits);

        m_configurator.apply(configuration, CtreUtils.k_defaultConfiguratorTimeout.asSeconds());

        BaseStatusSignal.setUpdateFrequencyForAll(50, m_motor.getPosition(), m_motor.getVelocity());
        BaseStatusSignal.setUpdateFrequencyForAll(
                20,
                m_motor.getAcceleration(),
                m_motor.getSupplyVoltage(),
                m_motor.getMotorVoltage(),
                m_motor.getSupplyCurrent(),
                m_motor.getStatorCurrent());
        m_motor.optimizeBusUtilization();

        // -----------------------
        //    Actuator Settings
        // -----------------------
        m_actuator = TalonFxActuator.motorActuator(m_motor);

        // -------------
        //    Logging
        // -------------
        // m_logPower =
        //         builder.makeSyncLogEntry("Collector/Roller/Power", builder.voltageType_volts,
        // Voltage.ZERO);
        // m_logSupplyCurrent = builder.makeSyncLogEntry(
        //         "Collector/Roller/Supply Current", builder.currentType_amps, Current.ZERO);
        // m_logStatorCurrent = builder.makeSyncLogEntry(
        //         "Collector/Roller/Stator Current", builder.currentType_amps, Current.ZERO);
    }

    @Override
    public void update() {
        // Logging
        // m_logPower.accept(Voltage.volts(m_motor.getMotorVoltage().getValueAsDouble()));
        // m_logSupplyCurrent.accept(Current.amps(m_motor.getSupplyCurrent().getValueAsDouble()));
        // m_logStatorCurrent.accept(Current.amps(m_motor.getStatorCurrent().getValueAsDouble()));
    }

    @Override
    public void setPower(Voltage power) {
        m_actuator.setOutputVoltage(power);
    }

    @Override
    public void setIntendedNeutralMode() {
        m_motor.setControl(CtreUtils.getControlRequest(k_intendedNeutralMode));
    }

    @Override
    public void brake() {
        m_motor.setControl(new StaticBrake());
    }

    @Override
    public void coast() {
        m_motor.setControl(new CoastOut());
    }
}
