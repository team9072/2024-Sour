package frc.robot.subsystems.intake.deploy;

import org.growingstems.control.actuators.MotorActuator;
import org.growingstems.control.actuators.PositionActuator;
import org.growingstems.frc.actuators.TalonFxActuator;
import org.growingstems.measurements.Measurements.Current;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.library.CtreUtils;
import frc.robot.Robot;

public class IntakeDeployHal implements IntakeDeployHalI {

    // Motor
    public final TalonFX m_motor;
    private final TalonFXConfigurator m_configurator;

    // Motor Configuration
    private static final int k_canId = 14;
    private static final NeutralModeValue k_intendedNeutralMode = NeutralModeValue.Brake;
    // CounterClockwise_Positive is the default value of a TalonFX
    private static final InvertedValue k_invertedSetting = InvertedValue.Clockwise_Positive;
    
    private static final Current k_supplyCurrentLimit = Current.amps(25.0);
    private static final Current k_statorCurrentLimit = Current.amps(120.0);
    
    // PID Configs
    private static final double k_sensorToMechanism = 49.7;
    private static final double k_g = 0.6;


    public IntakeDeployHal() {
        
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

        var feedback = new FeedbackConfigs();
        feedback.SensorToMechanismRatio = k_sensorToMechanism;
        configuration.withFeedback(feedback);
        
        var pid = new Slot0Configs();
        pid.GravityType = GravityTypeValue.Arm_Cosine;
        pid.kG = k_g;
        configuration.withSlot0(pid);

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

    }

    @Override
    public void setPosition(double degrees) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public double setHomePoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setHomePoint'");
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
    }

    @Override
    public double getError() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getError'");
    }
    
}
