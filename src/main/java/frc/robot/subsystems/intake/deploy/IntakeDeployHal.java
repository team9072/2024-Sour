package frc.robot.subsystems.intake.deploy;

import org.growingstems.measurements.Measurements.Current;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.library.CtreUtils;
import frc.robot.Robot;

public class IntakeDeployHal implements IntakeDeployHalI {

    // Motor
    private final TalonFX m_motor;
    private final MotionMagicVoltage m_control;
    private final TalonFXConfigurator m_configurator;

    // Motor Configuration
    private static final int k_canId = 14;
    private static final NeutralModeValue k_intendedNeutralMode = NeutralModeValue.Brake;
    // CounterClockwise_Positive is the default value of a TalonFX
    private static final InvertedValue k_invertedSetting = InvertedValue.Clockwise_Positive;
    
    private static final Current k_supplyCurrentLimit = Current.amps(35.0);
    private static final Current k_statorCurrentLimit = Current.amps(120.0);
    
    // PID Configs
    private static final double k_sensorToMechanism = 49.7;
    private static final double k_realHomeRotations = 0.15;
    private static final double kG = 0.15;
    private static final double kP = 20;
    private static final double kV = 6;
    
    private static final double k_acceleration = 3;
    private static final double k_velocity = 4;

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
        pid.kG = kG;
        pid.kP = kP;
        pid.kV= kV;
        configuration.withSlot0(pid);

        var motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicAcceleration = k_acceleration;
        motionMagic.MotionMagicCruiseVelocity = k_velocity;
        configuration.withMotionMagic(motionMagic);

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

        setHomePoint();

        m_control = new MotionMagicVoltage(k_realHomeRotations);
        m_control.EnableFOC = false;
        m_control.Slot = 0;
    }

    @Override
    public void setPosition(double positionDegrees) {
        m_control.Position = Units.degreesToRotations(positionDegrees);
        m_motor.setControl(m_control);
    }
    
    @Override
    public void setHomePoint() {
        m_motor.setPosition(k_realHomeRotations, CtreUtils.k_defaultConfiguratorTimeout.asSeconds());
    }

    @Override
    public double getPosition() {
        return Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble());
    }

    @Override
    public double getVelocity() {
        return Units.rotationsToDegrees(m_motor.getVelocity().getValueAsDouble());
    }
    
}
