/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.tracking;

import java.util.List;
import org.growingstems.math.Vector2d;
import org.growingstems.math.Vector2dU;
import org.growingstems.math.Vector3dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonTracker implements PieceTracker {
    protected final PhotonCamera m_camera;
    protected final Vector3dU<Length> m_transform;
    protected final Vector3dU<Angle> m_ypr;

    public PhotonTracker(String name, Vector3dU<Length> transform, Vector3dU<Angle> ypr) {
        m_camera = new PhotonCamera(name);
        m_transform = transform;
        m_ypr = ypr;
    }

    @Override
    public List<Vector2dU<Length>> getRobotRelativeDeltas() {
        return m_camera.getLatestResult().getTargets().stream()
                .map(t -> new Vector2d(t.getYaw(), t.getPitch()))
                .map(v -> v.forEachU(Angle::degrees))
                .map(v -> {
                    var targetYaw = m_ypr.getX().add(v.getX().neg()); // Left-Positive
                    var targetPitch = m_ypr.getY().neg().add(v.getY()); // Up-Positive
                    var range = m_transform.getZ().div(targetPitch.neg().tan()); // In the XY Plane
                    return Vector2dU.fromPolar(range, targetYaw).add(m_transform.getXY());
                })
                .toList();
    }

    @Override
    public List<Angle> getTargetPlaneAngles() {
        return m_camera.getLatestResult().getTargets().stream()
                .map(PhotonTrackedTarget::getYaw)
                .map(Angle::degrees)
                .toList();
    }
}
