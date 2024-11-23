/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.logging.LogBuilder;
import frc.robot.robotpose.PoseProvider;
import frc.robot.subsystems.drive.CtreSwerveDrive;
import frc.robot.subsystems.drive.SwerveDriveI;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.SourDriveConstants;
import java.util.function.Supplier;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Frequency;
import org.growingstems.measurements.Measurements.FrequencySquared;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Measurements.Unitless;
import org.growingstems.measurements.Measurements.Velocity;

public class Drive extends SubsystemBase {
    private final SwerveDriveI m_swerveDrive;
    private final PoseProvider m_poseProvider;

    // PID constants
    private static final double k_pidPeriodApproximation = 0.02;

    // Angular Velocity per Angle-Error
    private static final Frequency k_headingPid_p =
            AngularVelocity.revolutionsPerSecond(1.0).div(Angle.degrees(72.0));
    // Angular Velocity per Angle-Error/Time
    private static final Unitless k_headingPid_d =
            AngularVelocity.revolutionsPerSecond(0.0).div(AngularVelocity.revolutionsPerSecond(1.0));

    // Velocity per Length-Error
    private static final Frequency k_translationPid_p =
            Velocity.inchesPerSecond(8.0).div(Length.inches(1.0));
    // Velocity per Length-Error*Time
    private static final FrequencySquared k_translationPid_i =
            Velocity.inchesPerSecond(0.0).div(Length.inches(1.0).mul(Time.seconds(1.0)));
    // Velocity per Length-Error/Time
    private static final Unitless k_translationPid_d =
            Velocity.inchesPerSecond(0.0).div(Velocity.inchesPerSecond(1.0));

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
}
