/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import frc.library.drivers.ctre.CtreUtils;
import frc.library.utils.ConversionUtils;
import frc.robot.logging.LogBuilder;
import frc.robot.subsystems.drive.constants.DriveConstants;
import org.growingstems.frc.util.WpiTimeSource;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Velocity;
import org.growingstems.util.timer.Timer;

public class CtreSimDrive extends CtreSwerveDrive {
    private final Timer m_timer;

    protected final SwerveDrivetrain m_ctreOdometryReference;

    private static final NeutralModeValue k_intendedNeutralMode = NeutralModeValue.Brake;

    public CtreSimDrive(
            LogBuilder logBuilder,
            SwerveDrivetrainConstants driveTrainConstants,
            DriveConstants moduleConstants) {
        super(logBuilder, driveTrainConstants, moduleConstants);
        m_timer = new WpiTimeSource().createTimer();
        m_timer.reset();
        m_timer.start();

        var moduleArray = moduleConstants
                .asList()
                .toArray(new SwerveModuleConstants[moduleConstants.asList().size()]);
        m_ctreOdometryReference =
                new SwerveDrivetrain(driveTrainConstants, k_odometryUpdateRate.asHertz(), moduleArray);
    }

    @Override
    public void stop() {
        var request = new SwerveRequest.ApplyChassisSpeeds();
        request.Speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        request.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        request.SteerRequestType = k_steerControllerMode;

        m_ctreDrive.setControl(request);
        m_ctreOdometryReference.setControl(request);
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
        m_ctreOdometryReference.setControl(request);
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
        m_ctreOdometryReference.setControl(request);
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
        // request.HeadingController.setP(k_headingPid.p); // TODO: Figure out PID constants
        // request.HeadingController.setI(k_headingPid.i.asHertz()); // TODO: Figure out PID constants
        // request.HeadingController.setD(k_headingPid.d.asSeconds()); // TODO: Figure out PID constants
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        m_ctreDrive.setControl(request);
        m_ctreOdometryReference.setControl(request);
    }

    @Override
    public void xStop() {
        m_ctreDrive.setControl(new SwerveRequest.SwerveDriveBrake());
        m_ctreOdometryReference.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    @Override
    public void pointModulesAt(Angle moduleAngle) {
        var request = new SwerveRequest.PointWheelsAt();
        request.ModuleDirection = ConversionUtils.toWpi(moduleAngle);
        m_ctreDrive.setControl(request);
        m_ctreOdometryReference.setControl(request);
    }

    @Override
    public void setDriveCoast() {
        for (var module : m_modules) {
            module.getDriveMotor().setControl(new CoastOut());
        }
    }

    @Override
    public void setDriveBrake() {
        for (var module : m_modules) {
            module.getDriveMotor().setControl(CtreUtils.getControlRequest(k_intendedNeutralMode));
        }
    }

    @Override
    public void setPose(Pose2dU<Length> pose) {
        m_ctreDrive.seedFieldRelative(ConversionUtils.toWpiMeters(pose));
        m_ctreOdometryReference.seedFieldRelative(ConversionUtils.toWpiMeters(pose));
    }

    @Override
    public Vector2dU<Velocity> getFieldVelocity() {
        return getVelocityVector().rotate(getPose().getRotation());
    }

    @Override
    public AngularVelocity getYawRate() {
        return AngularVelocity.degreesPerSecond(
                m_ctreOdometryReference.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    }

    @Override
    public Angle getPitch() {
        return Angle.degrees(m_ctreOdometryReference.getPigeon2().getPitch().getValueAsDouble());
    }

    @Override
    public Angle getRoll() {
        return Angle.degrees(m_ctreOdometryReference.getPigeon2().getRoll().getValueAsDouble());
    }

    public void updatePose() {
        var dt = m_timer.get().asSeconds();
        m_timer.reset();

        m_ctreDrive.updateSimState(dt, RobotController.getBatteryVoltage());
        m_ctreOdometryReference.updateSimState(dt, RobotController.getBatteryVoltage());
    }
}
