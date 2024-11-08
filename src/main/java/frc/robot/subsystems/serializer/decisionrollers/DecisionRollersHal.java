package frc.robot.subsystems.serializer.decisionrollers;

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
import frc.robot.logging.LogBuilder;
import java.util.function.Consumer;
import org.growingstems.control.actuators.MotorActuator;
import org.growingstems.frc.actuators.TalonFxActuator;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.Voltage;

public class DecisionRollersHal implements DecisionRollersHalI {

    // Motor
    private final MotorActuator m_actuator;
    private final TalonFX m_motor;
    private final TalonFXConfigurator m_configurator;

    // Motor Configuration
    private static final int k_canId = 17;
    private static final NeutralModeValue k_intendedNeutralMode = NeutralModeValue.Brake;
    // CounterClockwise_Positive is the default value of a TalonFX
    private static final InvertedValue k_invertedSetting = InvertedValue.CounterClockwise_Positive;
    private static final Current k_supplyCurrentLimit = Current.amps(25.0);
    private static final Current k_statorCurrentLimit = Current.amps(120.0);

    //

    // Logging
    private final Consumer<Voltage> m_logPower;
    private final Consumer<Current> m_logSupplyCurrent;
    private final Consumer<Current> m_logStatorCurrent;

    public DecisionRollersHal(LogBuilder builder) {
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
        general.Inverted = k_invertedSetting;
        configuration.withMotorOutput(general);

        // Current Limiting
        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = k_supplyCurrentLimit.asAmps();
        currentLimits.StatorCurrentLimit = k_statorCurrentLimit.asAmps();
        configuration.withCurrentLimits(currentLimits);

        m_configurator.apply(configuration, CtreUtils.k_defaultConfiguratorTimeout.asSeconds());

        // ----------------------------
        //    Status Signal Settings
        // ----------------------------
        // 100 Hz Rate (Every 10ms)
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                m_motor.getPosition(),
                m_motor.getVelocity(),
                m_motor.getAcceleration(),
                m_motor.getSupplyVoltage(),
                m_motor.getMotorVoltage(),
                m_motor.getSupplyCurrent(),
                m_motor.getTorqueCurrent());
        // 4 Hz Rate (every 100ms)
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                m_motor.getProcessorTemp(),
                m_motor.getDeviceTemp(),
                m_motor.getAncillaryDeviceTemp(),
                m_motor.getFaultField(),
                m_motor.getStickyFaultField());
        m_motor.optimizeBusUtilization();

        // -----------------------
        //    Actuator Settings
        // -----------------------
        m_actuator = TalonFxActuator.motorActuator(m_motor);

        // -------------
        //    Logging
        // -------------
        m_logPower = builder.makeSyncLogEntry("Serializer/Power", builder.voltageType_volts);
        m_logSupplyCurrent =
                builder.makeSyncLogEntry("Serializer/Supply Current", builder.currentType_amps);
        m_logStatorCurrent =
                builder.makeSyncLogEntry("Serializer/Stator Current", builder.currentType_amps);
    }

    @Override
    public void update() {
        // Logging
        m_logPower.accept(Voltage.volts(m_motor.getMotorVoltage().getValueAsDouble()));
        m_logSupplyCurrent.accept(Current.amps(m_motor.getSupplyCurrent().getValueAsDouble()));
        m_logStatorCurrent.accept(Current.amps(m_motor.getStatorCurrent().getValueAsDouble()));
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
