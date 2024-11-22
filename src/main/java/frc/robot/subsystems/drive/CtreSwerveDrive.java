/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.library.actuators.AngularPidf;
import frc.library.drivers.ctre.CtreUtils;
import frc.library.swerve.SwerveDriveBrake;
import frc.library.utils.ConversionUtils;
import frc.robot.logging.LogBuilder;
import frc.robot.pose.PoseProvider;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.vision.VisionSystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.Frequency;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Velocity;
import org.growingstems.measurements.Measurements.Voltage;

public class CtreSwerveDrive implements SwerveDriveI, PoseProvider {
    protected final SwerveDrivetrain m_ctreDrive;
    protected final VisionSystem m_visionSystem;

    // TODO: Remove when CTRE provides a more convenient solution
    // https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/73
    protected final List<SwerveModule> m_modules = new ArrayList<SwerveModule>();
    protected boolean m_enable = true;

    // A odometry update rate value of 0.0 means let it decide
    protected static final Frequency k_odometryUpdateRate = Frequency.hertz(0.0);

    protected static final SteerRequestType k_steerControllerMode = SteerRequestType.MotionMagic;

    // PID constants
    protected static final AngularPidf k_headingPid = new AngularPidf().withP(5.0);

    // Deadbands
    protected static final Velocity k_translationDeadband = Velocity.inchesPerSecond(0.1);
    protected static final AngularVelocity k_rotationDeadband = AngularVelocity.degreesPerSecond(1.0);

    // Logger
    private Consumer<Vector2dU<Velocity>> m_logVelocityVector;

    private Consumer<Current> m_logDriveSupplySumCurrent;
    private Consumer<List<Double>> m_logDriveSupplyCurrent;

    private Consumer<Current> m_logDriveStatorSumCurrent;
    private Consumer<List<Double>> m_logDriveStatorCurrent;

    private Consumer<List<Double>> m_logDriveVoltageOut;

    public CtreSwerveDrive(
            LogBuilder builder,
            SwerveDrivetrainConstants driveTrainConstants,
            DriveConstants moduleConstants,
            VisionSystem visionSystem) {
        // Create CTRE Swerve Drive
        var moduleArray = moduleConstants
                .asList()
                .toArray(new SwerveModuleConstants[moduleConstants.asList().size()]);
        m_ctreDrive =
                new SwerveDrivetrain(driveTrainConstants, k_odometryUpdateRate.asHertz(), moduleArray);

        var allMotors = new ArrayList<TalonFX>();
        // TODO: Remove when CTRE provides a more convenient solution
        // https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/73
        for (int i = 0; i < moduleConstants.asList().size(); i++) {
            var module = m_ctreDrive.getModule(i);
            m_modules.add(module);

            // Current Limiting
            var currentLimits = new CurrentLimitsConfigs();
            currentLimits.SupplyCurrentLimitEnable = true;
            currentLimits.SupplyCurrentLimit = Current.amps(60.0).asAmps();
            currentLimits.StatorCurrentLimitEnable = true;
            currentLimits.StatorCurrentLimit = Current.amps(180.0).asAmps();
            module.getDriveMotor().getConfigurator().apply(currentLimits);

            // Music
            allMotors.add(module.getDriveMotor());
            allMotors.add(module.getSteerMotor());
        }

        // Music
        for (var motor : allMotors) {
            var audio = new AudioConfigs();
            audio.AllowMusicDurDisable = true;
            motor.getConfigurator().apply(audio);

            // Robot.orchestra.addInstrument(motor);
        }

        m_visionSystem = visionSystem;

        // Logger
        // TODO: Implement for 9072
        // m_logVelocityVector = builder.makeSyncLogEntry(
        //         "Swerve/Velocity Vector",
        //         builder.vector2dUType_polar_ftps,
        //         new Vector2dU<>(Velocity.ZERO, Velocity.ZERO));

        // m_logDriveSupplySumCurrent = builder.makeSyncLogEntry(
        //         "Swerve/Drive Supply Current Sum", builder.currentType_amps, Current.ZERO);
        // m_logDriveSupplyCurrent = builder.makeSyncLogEntry(
        //         "Swerve/Drive Supply Current(amps)",
        //         builder.toList(builder.doubleType),
        //         new ArrayList<Double>(4));

        // m_logDriveStatorSumCurrent = builder.makeSyncLogEntry(
        //         "Swerve/Drive Stator Current Sum", builder.currentType_amps, Current.ZERO);
        // m_logDriveStatorCurrent = builder.makeSyncLogEntry(
        //         "Swerve/Drive Stator Current(amps)",
        //         builder.toList(builder.doubleType),
        //         new ArrayList<Double>(4));

        // m_logDriveVoltageOut = builder.makeSyncLogEntry(
        //         "Swerve/Drive Voltage Out(V)",
        //         builder.toList(builder.doubleType),
        //         new ArrayList<Double>(4));
    }

    public void update() {
        m_logVelocityVector.accept(getVelocityVector());

        var totalSupplyCurrent = Current.ZERO;
        var supplyCurrents = new ArrayList<Double>(m_modules.size());
        var totalStatorCurrent = Current.ZERO;
        var statorCurrents = new ArrayList<Double>(m_modules.size());
        var outputVoltages = new ArrayList<Double>(m_modules.size());

        for (var module : m_modules) {
            var supplyCurrent = Current.amps(module.getDriveMotor().getSupplyCurrent().getValue());
            totalSupplyCurrent.add(supplyCurrent);
            supplyCurrents.add(supplyCurrent.asAmps());

            var statorCurrent = Current.amps(module.getDriveMotor().getStatorCurrent().getValue());
            totalStatorCurrent.add(statorCurrent);
            statorCurrents.add(statorCurrent.asAmps());

            outputVoltages.add(Voltage.volts(module.getDriveMotor().getMotorVoltage().getValueAsDouble())
                    .asVolts());
        }

        m_logDriveSupplySumCurrent.accept(totalSupplyCurrent);
        m_logDriveSupplyCurrent.accept(supplyCurrents);

        m_logDriveStatorSumCurrent.accept(totalStatorCurrent);
        m_logDriveStatorCurrent.accept(statorCurrents);

        m_logDriveVoltageOut.accept(outputVoltages);
    }

    @Override
    public void configForSysIdRoutine() {
        // Configure all modules for SysID analysis
        for (var module : m_modules) {
            var motor = module.getDriveMotor();

            // Reset Back to Factory Default
            var configurator = motor.getConfigurator();
            configurator.apply(
                    new TalonFXConfiguration(), CtreUtils.k_defaultConfiguratorTimeout.asSeconds());

            // Speed up signals for better characterization data
            BaseStatusSignal.setUpdateFrequencyForAll(
                    250, motor.getPosition(), motor.getVelocity(), motor.getMotorVoltage());

            // Optimize out the other signals, since they're not particularly helpful for us
            motor.optimizeBusUtilization();
        }

        SignalLogger.start();
    }

    /**
     * Gets the drive's current velocity as a vector relative to the robot.
     *
     * @return Current velocity of the drive
     */
    @Override
    public Vector2dU<Velocity> getVelocityVector() {
        var velocityVector = new Vector2dU<Velocity>(Velocity.ZERO, Velocity.ZERO);

        for (var module : m_modules) {
            var state = module.getCurrentState();
            var velocity = Velocity.metersPerSecond(state.speedMetersPerSecond);
            var direction = ConversionUtils.fromWpi(state.angle);

            velocityVector = velocityVector.add(Vector2dU.fromPolar(velocity, direction));
        }

        return velocityVector.div(m_modules.size());
    }

    @Override
    public void stop() {
        var request = new SwerveRequest.ApplyChassisSpeeds();
        request.Speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        request.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        request.SteerRequestType = k_steerControllerMode;

        m_ctreDrive.setControl(request);
    }

    @Override
    public void setRobotCentric(
            Vector2dU<Velocity> translation, AngularVelocity rotation, DriveRequestType driveMode) {
        if (!m_enable) {
            stop();
            return;
        }

        var request = new SwerveRequest.RobotCentric();
        request.VelocityX = translation.getX().asMetersPerSecond();
        request.VelocityY = translation.getY().asMetersPerSecond();
        request.RotationalRate = rotation.asRadiansPerSecond();
        request.DriveRequestType = driveMode;
        request.SteerRequestType = k_steerControllerMode;
        request.Deadband = k_translationDeadband.asMetersPerSecond();
        request.RotationalDeadband = k_rotationDeadband.asRadiansPerSecond();

        m_ctreDrive.setControl(request);
    }

    @Override
    public void setFieldCentric(
            Vector2dU<Velocity> translation, AngularVelocity rotation, DriveRequestType driveMode) {
        if (!m_enable) {
            stop();
            return;
        }

        var request = new SwerveRequest.FieldCentric();
        request.VelocityX = translation.getX().asMetersPerSecond();
        request.VelocityY = translation.getY().asMetersPerSecond();
        request.RotationalRate = rotation.asRadiansPerSecond();
        request.DriveRequestType = driveMode;
        request.SteerRequestType = k_steerControllerMode;
        request.Deadband = k_translationDeadband.asMetersPerSecond();
        request.RotationalDeadband = k_rotationDeadband.asRadiansPerSecond();

        m_ctreDrive.setControl(request);
    }

    @Override
    public void setFieldCentric(
            Vector2dU<Velocity> translation, Angle heading, DriveRequestType driveMode) {
        if (!m_enable) {
            stop();
            return;
        }

        // TODO: Since this request has a PID Controller in it, it should be a member variable
        var request = new SwerveRequest.FieldCentricFacingAngle();
        request.VelocityX = translation.getX().asMetersPerSecond();
        request.VelocityY = translation.getY().asMetersPerSecond();
        request.TargetDirection = ConversionUtils.toWpi(heading);
        request.DriveRequestType = driveMode;
        request.SteerRequestType = k_steerControllerMode;
        request.Deadband = k_translationDeadband.asMetersPerSecond();
        request.RotationalDeadband = k_rotationDeadband.asRadiansPerSecond();

        // Heading Controller PID Settings
        request.HeadingController.setP(k_headingPid.p);
        request.HeadingController.setI(k_headingPid.i.asHertz());
        request.HeadingController.setD(k_headingPid.d.asSeconds());
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        m_ctreDrive.setControl(request);
    }

    @Override
    public void xStop() {
        m_ctreDrive.setControl(new SwerveDriveBrake());
    }

    @Override
    /** Puts the wheels in a position that would rotate the robot, but does not apply motor power. */
    public void idleRotate() {
        m_ctreDrive.setControl(
                new SwerveRequest.SysIdSwerveRotation().withVolts(ConversionUtils.toWpi(Voltage.ZERO)));
    }

    public void pointModulesAt(Angle moduleAngle) {
        var request = new SwerveRequest.PointWheelsAt();
        request.ModuleDirection = ConversionUtils.toWpi(moduleAngle);
        m_ctreDrive.setControl(request);
    }

    public void setDriveCoast() {
        for (var module : m_modules) {
            module.configNeutralMode(NeutralModeValue.Coast);
        }
    }

    public void setDriveBrake() {
        for (var module : m_modules) {
            module.configNeutralMode(NeutralModeValue.Brake);
        }
    }

    // *****************************
    //    Pose Provider Functions
    // *****************************
    @Override
    public Pose2dU<Length> getPose() {
        return ConversionUtils.fromWpi(m_ctreDrive.getState().Pose);
    }

    @Override
    public void setPose(Pose2dU<Length> pose) {
        m_ctreDrive.seedFieldRelative(ConversionUtils.toWpiMeters(pose));
    }

    @Override
    public Vector2dU<Velocity> getFieldVelocity() {
        return getVelocityVector().rotate(getPose().getRotation());
    }

    @Override
    public AngularVelocity getYawRate() {
        return AngularVelocity.degreesPerSecond(
                m_ctreDrive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    }

    @Override
    public Angle getPitch() {
        return Angle.degrees(m_ctreDrive.getPigeon2().getPitch().getValueAsDouble());
    }

    @Override
    public Angle getRoll() {
        return Angle.degrees(m_ctreDrive.getPigeon2().getRoll().getValueAsDouble());
    }

    @Override
    public void updatePose() {
        for (var result : m_visionSystem.getResults(this)) {
            m_ctreDrive.addVisionMeasurement(
                    ConversionUtils.toWpiMeters(result.visionRobotPose),
                    result.timestamp.asSeconds(),
                    result.visionMeasurementStdDevs);
        }
    }
}
