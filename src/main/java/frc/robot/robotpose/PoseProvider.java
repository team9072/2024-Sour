/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.robotpose;

import org.growingstems.math.Pose2dU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Velocity;

public interface PoseProvider {
    Pose2dU<Length> getPose();

    void setPose(Pose2dU<Length> pose);

    Vector2dU<Velocity> getFieldVelocity();

    AngularVelocity getYawRate();

    Angle getPitch();

    Angle getRoll();

    void updatePose();

    default void setYaw(Angle yaw) {
        setPose(new Pose2dU<>(getPose().getVector(), yaw));
    }
}
