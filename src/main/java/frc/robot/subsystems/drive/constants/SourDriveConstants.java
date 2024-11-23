/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.drive.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Measurements.Velocity;
import org.growingstems.measurements.Measurements.Voltage;

public class SourDriveConstants extends DriveConstants {
    protected static final int k_pigeon2CanId = 1;
    protected static final Length k_modulePerimeterWidth = Length.inches(20.75);
    protected static final Length k_modulePerimeterLength = Length.inches(20.75);

    protected Angle getFrontLeftOffset() {
        return Angle.revolutions((-0.168701 * 360.0) - 180.0);
    }

    protected Angle getBackLeftOffset() {
        return Angle.revolutions((0.141357 * 360.0) - 180.0);
    }

    protected Angle getBackRightOffset() {
        return Angle.revolutions(-0.074463 * 360.0);
    }

    protected Angle getFrontRightOffset() {
        return Angle.revolutions(0.952148 * 360.0);
    }

    // **************************************
    //    Generated Code from CTRE Tuner X
    //        Cleaned up to match our
    //       styleguide and added units
    // **************************************

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs k_steerGains = new Slot0Configs()
            .withKP(1.005)
            .withKI(0.0)
            .withKD(0.0004)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType k_steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType k_driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current k_slipCurrent = Current.amps(180.0);

    // Every 1 rotation of the azimuth results in k_CoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double k_coupleRatio = 3.5;

    private static final double k_driveGearRatio = 5.82;
    private static final double k_steerGearRatio = (150.0 / 7.0);

    private static final boolean k_steerMotorReversed = true;

    // These are only used for simulation
    private static final double k_steerInertia = 0.0001;
    private static final double k_driveInertia = 0.005;
    // Simulated voltage necessary to overcome friction
    private static final Voltage k_steerFriction = Voltage.volts(0.35);
    private static final Voltage k_driveFriction = Voltage.volts(0.35);

    // *********************************************
    //    End of Generated Code from CTRE Tuner X
    // *********************************************

    /** Generated with SysID */
    private static final Slot0Configs k_driveGains = new Slot0Configs()
            .withKP(0.030 * 12.0)
            .withKI(0.0)
            .withKD(0.000001 * 12.0)
            .withKS(0.1)
            .withKV(0.0)
            .withKA(0.0);

    private static final Length k_wheelRadius = Length.inches(2.0);

    /**
     * This value was found by empirically by setting {@code k_speedAt12Volts} to 12.0, and then using
     * {@code CtreSwerveDrive}'s {@code setRobotCentric} to force the drive to move in the +/- X
     * direction. Voltage values were translated directly from volts to meters per second (since
     * "speed at 12V" was set to 12.0). Using SignalLogger (could use any logger), log the measured
     * output voltage and velocity of the drive motors. Using a log viewing tool (AdvantageScope),
     * verify what the typical max speed while 12V is being applied in meters per second.
     */
    private static final AngularVelocity driveMotorMaxRotationAt12V =
            Angle.revolutions(100.0).div(Time.seconds(1.0));

    private static final AngularVelocity wheelMaxRotationAt12V =
            driveMotorMaxRotationAt12V.div(k_driveGearRatio);

    /** Theoretical free speed (m/s) at 12v applied output */
    private static final Velocity k_speedAt12Volts = wheelMaxRotationAt12V.mul(k_wheelRadius);

    protected SwerveModuleConstantsFactory m_factory;

    public SourDriveConstants() {
        super(k_pigeon2CanId);
    }

    @Override
    protected void init() {
        m_factory = new SwerveModuleConstantsFactory();

        // Drive
        m_factory.withDriveMotorGains(k_driveGains);
        m_factory.withCouplingGearRatio(k_coupleRatio);
        m_factory.withDriveMotorClosedLoopOutput(k_driveClosedLoopOutput);
        m_factory.withDriveMotorGearRatio(k_driveGearRatio);
        m_factory.withSlipCurrent(k_slipCurrent.asAmps());
        m_factory.withSpeedAt12VoltsMps(k_speedAt12Volts.asMetersPerSecond());
        // Code that uses this is in SwerveModule.class:334 at the time of writing this comment.

        // Steer
        m_factory.withFeedbackSource(SteerFeedbackType.FusedCANcoder);
        m_factory.withSteerMotorGains(k_steerGains);
        m_factory.withSteerMotorClosedLoopOutput(k_steerClosedLoopOutput);
        m_factory.withSteerMotorGearRatio(k_steerGearRatio);
        m_factory.withSteerMotorInverted(k_steerMotorReversed);
        m_factory.withWheelRadius(k_wheelRadius.asInches());

        // Simulation
        m_factory.SteerInertia = k_steerInertia;
        m_factory.DriveInertia = k_driveInertia;
        m_factory.SteerFrictionVoltage = k_steerFriction.asVolts();
        m_factory.DriveFrictionVoltage = k_driveFriction.asVolts();
    }

    @Override
    protected SwerveModuleConstants createFrontLeftConstants() {
        Vector2dU<Length> modulePos =
                new Vector2dU<Length>(k_modulePerimeterLength.div(2.0), k_modulePerimeterWidth.div(2.0));

        return m_factory.createModuleConstants(
                3,
                2,
                4,
                getFrontLeftOffset().asRevolutions(),
                modulePos.getX().asMeters(),
                modulePos.getY().asMeters(),
                false);
    }

    @Override
    protected SwerveModuleConstants createBackLeftConstants() {
        Vector2dU<Length> modulePos = new Vector2dU<Length>(
                k_modulePerimeterLength.neg().div(2.0), k_modulePerimeterWidth.div(2.0));

        return m_factory.createModuleConstants(
                9,
                8,
                10,
                getBackLeftOffset().asRevolutions(),
                modulePos.getX().asMeters(),
                modulePos.getY().asMeters(),
                false);
    }

    @Override
    protected SwerveModuleConstants createBackRightConstants() {
        Vector2dU<Length> modulePos = new Vector2dU<Length>(
                k_modulePerimeterLength.neg().div(2.0), k_modulePerimeterWidth.neg().div(2.0));

        return m_factory.createModuleConstants(
                12,
                11,
                13,
                getBackRightOffset().asRevolutions(),
                modulePos.getX().asMeters(),
                modulePos.getY().asMeters(),
                false);
    }

    @Override
    protected SwerveModuleConstants createFrontRightConstants() {
        Vector2dU<Length> modulePos = new Vector2dU<Length>(
                k_modulePerimeterLength.div(2.0), k_modulePerimeterWidth.neg().div(2.0));

        return m_factory.createModuleConstants(
                6,
                5,
                7,
                getFrontRightOffset().asRevolutions(),
                modulePos.getX().asMeters(),
                modulePos.getY().asMeters(),
                false);
    }
}
