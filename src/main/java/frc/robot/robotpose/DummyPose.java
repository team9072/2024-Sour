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

// Dummy class for fake pose providing
public class DummyPose implements PoseProvider {
    private Pose2dU<Length> m_mem = new Pose2dU<Length>(Length.ZERO, Length.ZERO, Angle.ZERO);

    @Override
    public Pose2dU<Length> getPose() {
        return m_mem;
    }

    @Override
    public void setPose(Pose2dU<Length> pose) {
        m_mem = pose;
    }

    @Override
    public Vector2dU<Velocity> getFieldVelocity() {
        return new Vector2dU<Velocity>(Velocity.ZERO, Velocity.ZERO);
    }

    @Override
    public AngularVelocity getYawRate() {
        return AngularVelocity.ZERO;
    }

    @Override
    public void updatePose() {
        // NOP
    }

    @Override
    public Angle getPitch() {
        return Angle.ZERO;
    }

    @Override
    public Angle getRoll() {
        return Angle.ZERO;
    }
}
