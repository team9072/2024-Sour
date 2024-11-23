/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.library.utils.ConversionUtils;
import frc.robot.Robot;
import frc.robot.RobotRunner;
import frc.robot.logging.LogBuilder;
import frc.robot.robotpose.PoseProvider;
import frc.robot.subsystems.drive.CtreSwerveDrive;
import frc.robot.subsystems.drive.SwerveDriveI;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.SourDriveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.growingstems.frc.util.RobotMatchState;
import org.growingstems.frc.util.RobotMatchState.MatchMode;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Measurements.Velocity;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.signals.Memory;

public class Drive extends SubsystemBase {
    private final SwerveDriveI m_swerveDrive;
    private final PoseProvider m_poseProvider;

    public Drive(LogBuilder builder) {
        // CTRE Swerve Module Constants
        DriveConstants moduleConstants = new SourDriveConstants();

        var constants = moduleConstants;
        // CTRE Swerve Drive Constants
        var drivetrainConstants = new SwerveDrivetrainConstants();
        drivetrainConstants.CANbusName = Robot.k_canivoreCan;
        drivetrainConstants.Pigeon2Id = constants.getPigeon2CanId();

        var ctreDrive = new CtreSwerveDrive(builder, drivetrainConstants, constants);
        m_poseProvider = ctreDrive;
        m_swerveDrive = ctreDrive;
    }

    @Override
    public void periodic() {
        m_swerveDrive.update();
    }

    /**
     * Gets the drive's current velocity as a vector relative to the robot.
     *
     * @return Current velocity of the drive
     */
    public Vector2dU<Velocity> getVelocityVector() {
        return m_swerveDrive.getVelocityVector();
    }

    public PoseProvider getPoseProvider() {
        return m_poseProvider;
    }

    public Command stopICommand() {
        return new InstantCommand(() -> m_swerveDrive.xStop());
    }

    public Command getDirectDriveFCommand(
            Supplier<Vector2dU<Velocity>> velocity,
            Supplier<AngularVelocity> rotation,
            DriveRequestType driveMode) {
        return new FunctionalCommand(
                () -> {},
                () -> m_swerveDrive.setFieldCentric(velocity.get(), rotation.get(), driveMode),
                unused -> m_swerveDrive.stop(),
                () -> false,
                this);
    }

    public Command getHeadingPointDriveFCommand(
            Supplier<Angle> pointToFace,
            Supplier<Vector2dU<Velocity>> velocity,
            Supplier<AngularVelocity> turnOverride,
            DriveRequestType driveMode) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    var translation = velocity.get();
                    var rotation = turnOverride.get();
                    if (rotation.eq(AngularVelocity.ZERO)) {
                        m_swerveDrive.setFieldCentric(translation, pointToFace.get(), driveMode);
                    } else {
                        m_swerveDrive.setFieldCentric(translation, rotation, driveMode);
                    }
                },
                unused -> m_swerveDrive.stop(),
                () -> false,
                this);
    }

    public Command getFacePointDriveCommand(
            Supplier<Vector2dU<Length>> pointToFace,
            Supplier<Vector2dU<Velocity>> velocity,
            Supplier<AngularVelocity> turnOverride,
            DriveRequestType driveMode,
            Angle offset) {
        return getHeadingPointDriveFCommand(
                () -> pointToFace
                        .get()
                        .sub(m_poseProvider.getPose().getVector())
                        .getAngle()
                        .add(offset),
                velocity,
                turnOverride,
                driveMode);
    }

    public Command dollyFCommand(Supplier<Angle> direction) {
        return new InstantCommand(() -> m_swerveDrive.setDriveCoast(), this)
                .andThen(new RunCommand(() -> {
                    m_swerveDrive.pointModulesAt(direction.get());
                }))
                .finallyDo(() -> m_swerveDrive.setDriveBrake());
    }

    public Command panFCommand() {
        return new InstantCommand(() -> m_swerveDrive.setDriveCoast(), this)
                .andThen(new RunCommand(() -> {
                    m_swerveDrive.idleRotate();
                }))
                .finallyDo(() -> m_swerveDrive.setDriveBrake());
    }

    /**
     * <b>WARNING:</b> This command should only be used in an environment that allows for enough space
     * for the robot to drive forward and backwards, in a straight line, a significant distance
     * without running into anything/anyone. This command should <b>NEVER</b> be used during a
     * competition. If this command is ran while {@link RobotRunner#isCompetition()} is set true and
     * the robot isn't currently in TEST mode, this function will return an empty
     * {@link InstantCommand}.
     */
    public Command runSysIdRoutineUCommand(Time delayBetweenTests, Length testDistance) {
        BooleanSupplier okToRun = () -> !RobotRunner.isCompetition()
                && (RobotMatchState.getMatchState().matchMode == MatchMode.TEST);

        var sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Default ramp rate is acceptable
                        ConversionUtils.toWpi(
                                Voltage.volts(4.0)), // Reduce dynamic voltage to 4 to prevent motor brownout
                        null, // Default timeout is acceptable
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<edu.wpi.first.units.Voltage> volts) -> {
                            Velocity vel =
                                    Velocity.metersPerSecond(ConversionUtils.fromWpiVoltage(volts).asVolts());
                            var vector = Vector2dU.fromPolar(vel, Angle.ZERO);
                            m_swerveDrive.setRobotCentric(
                                    vector, AngularVelocity.ZERO, DriveRequestType.OpenLoopVoltage);
                        },
                        null,
                        this,
                        "Swerve Drive"));

        var startPosition = new Memory<Vector2dU<Length>>();
        Runnable resetStartPosition =
                () -> startPosition.update(m_poseProvider.getPose().getVector());
        Supplier<Command> resetStartPositionSupplier = () -> new InstantCommand(resetStartPosition);
        BooleanSupplier tooFar = () -> startPosition
                .getLastValue()
                .rangeTo(m_poseProvider.getPose().getVector())
                .ge(testDistance);
        return new InstantCommand(m_swerveDrive::configForSysIdRoutine)
                .andThen(
                        resetStartPositionSupplier.get(),
                        sysIdRoutine.dynamic(Direction.kForward).until(tooFar),
                        resetStartPositionSupplier.get(),
                        new WaitCommand(delayBetweenTests.asSeconds()),
                        sysIdRoutine.dynamic(Direction.kReverse).until(tooFar),
                        resetStartPositionSupplier.get(),
                        new WaitCommand(delayBetweenTests.asSeconds()),
                        sysIdRoutine.quasistatic(Direction.kForward).until(tooFar),
                        resetStartPositionSupplier.get(),
                        new WaitCommand(delayBetweenTests.asSeconds()),
                        sysIdRoutine.quasistatic(Direction.kReverse).until(tooFar),
                        stopICommand())
                .onlyIf(okToRun);
    }
}
