/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Velocity;

public interface SwerveDriveI {
    void setRobotCentric(
            Vector2dU<Velocity> translation, AngularVelocity rotation, DriveRequestType driveMode);

    void setFieldCentric(
            Vector2dU<Velocity> translation, AngularVelocity rotation, DriveRequestType driveMode);

    void setFieldCentric(Vector2dU<Velocity> translation, Angle heading, DriveRequestType driveMode);

    void xStop();

    void idleRotate();

    void stop();

    void pointModulesAt(Angle moduleAngle);

    void setDriveCoast();

    void setDriveBrake();

    void update();

    Vector2dU<Velocity> getVelocityVector();

    /**
     * This function should only be ran if and only if SysID is being used to characterize the
     * drivetrain. Once this function is ran, the only way to undo its effects is to restart robot
     * code.
     */
    void configForSysIdRoutine();
}
