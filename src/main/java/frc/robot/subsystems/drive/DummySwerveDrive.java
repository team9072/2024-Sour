/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Velocity;

public class DummySwerveDrive implements SwerveDriveI {
    private Vector2dU<Velocity> m_currentVelocityVector =
            new Vector2dU<>(Velocity.ZERO, Velocity.ZERO);

    @Override
    public void update() {
        // NOP
    }

    @Override
    public void setRobotCentric(
            Vector2dU<Velocity> translation, AngularVelocity rotation, DriveRequestType driveMode) {
        m_currentVelocityVector = translation;
    }

    @Override
    public void setFieldCentric(
            Vector2dU<Velocity> translation, AngularVelocity rotation, DriveRequestType driveMode) {
        m_currentVelocityVector = translation;
    }

    @Override
    public void setFieldCentric(
            Vector2dU<Velocity> translation, Angle heading, DriveRequestType driveMode) {
        m_currentVelocityVector = translation;
    }

    @Override
    public void xStop() {
        m_currentVelocityVector = new Vector2dU<>(Velocity.ZERO, Velocity.ZERO);
    }

    @Override
    public void idleRotate() {
        m_currentVelocityVector = new Vector2dU<>(Velocity.ZERO, Velocity.ZERO);
    }

    @Override
    public void stop() {
        m_currentVelocityVector = new Vector2dU<>(Velocity.ZERO, Velocity.ZERO);
    }

    @Override
    public void pointModulesAt(Angle moduleAngle) {
        m_currentVelocityVector = new Vector2dU<>(Velocity.ZERO, Velocity.ZERO);
    }

    @Override
    public void setDriveCoast() {
        // NOP
    }

    @Override
    public void setDriveBrake() {
        // NOP
    }

    @Override
    public Vector2dU<Velocity> getVelocityVector() {
        return m_currentVelocityVector;
    }

    @Override
    public void configForSysIdRoutine() {
        // NOP
    }
}
