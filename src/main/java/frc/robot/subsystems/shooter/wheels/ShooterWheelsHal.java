package frc.robot.subsystems.shooter.wheels;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.library.CtreUtils;
import frc.robot.Robot;
import org.growingstems.measurements.Measurements.Current;

public class ShooterWheelsHal implements ShooterWheelsHalI {
    // Motor
    private final TalonFX m_topMotor;
    private final TalonFX m_bottomMotor;
    private final TalonFXConfigurator m_topConfigurator;
    private final TalonFXConfigurator m_bottomConfigurator;

    private final NeutralOut m_neutralControl = new NeutralOut();
    private final VoltageOut m_voltageControl = new VoltageOut(0);
    private final MotionMagicVelocityVoltage m_velocityControl = new MotionMagicVelocityVoltage(0);
    private final Follower m_bottomControl = new Follower(k_topCanId, true);

    // Motor Configuration
    private static final int k_topCanId = 21;
    private static final int k_bottomCanId = 22;
    private static final NeutralModeValue k_intendedNeutralMode = NeutralModeValue.Coast;
    // CounterClockwise_Positive is the default value of a TalonFX
    private static final InvertedValue k_invertedSetting = InvertedValue.CounterClockwise_Positive;
    private static final Current k_supplyCurrentLimit = Current.amps(25.0);
    private static final Current k_statorCurrentLimit = Current.amps(120.0);

    public ShooterWheelsHal() {
        // --------------------
        //    Motor Settings
        // --------------------
        m_topMotor = new TalonFX(k_topCanId, Robot.k_canivoreCan);
        m_bottomMotor = new TalonFX(k_bottomCanId, Robot.k_canivoreCan);
        m_topConfigurator = m_topMotor.getConfigurator();
        m_bottomConfigurator = m_bottomMotor.getConfigurator();

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

        m_topConfigurator.apply(configuration, CtreUtils.k_defaultConfiguratorTimeout.asSeconds());
        m_bottomConfigurator.apply(configuration, CtreUtils.k_defaultConfiguratorTimeout.asSeconds());

        // ----------------------------
        //    Status Signal Settings
        // ----------------------------
        CtreUtils.configureDefaultSignals(m_topMotor);
        CtreUtils.configureDefaultSignals(m_bottomMotor);

        // Make bottom motor follow top
        m_bottomMotor.setControl(m_bottomControl);
    }

    public void coast() {
        m_topMotor.setControl(m_neutralControl);
        m_bottomMotor.setControl(m_bottomControl);
    }

    @Override
    public void setSpeed(double speedRpm) {
        m_bottomMotor.setControl(m_bottomControl);
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }

    @Override
    public void setVoltageSysId(double volts, boolean top) {
        if (top) {
            m_topMotor.setControl(m_voltageControl.withOutput(volts));
            m_bottomMotor.setControl(m_neutralControl);
        } else {
            m_bottomMotor.setControl(m_voltageControl.withOutput(volts));
            m_topMotor.setControl(m_neutralControl);
        }
    }

    @Override
    public double getTopSpeed() {
        return m_topMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getBottomSpeed() {
        return m_bottomMotor.getVelocity().getValueAsDouble();
    }
}
