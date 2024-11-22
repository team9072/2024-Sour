/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.posevision;

import edu.wpi.first.math.geometry.Transform3d;
import org.growingstems.math.Vector3dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class PhotonPoseSim extends PhotonPoseVision {

    private final PhotonCameraSim m_cameraSim;

    public PhotonPoseSim(
            String camera,
            SimCameraProperties simulatedCamProps,
            Vector3dU<Length> cameraPosition,
            Vector3dU<Angle> cameraYawPitchRoll,
            Length rejectionDist) {
        super(camera, cameraPosition, cameraYawPitchRoll, rejectionDist);
        m_cameraSim = new PhotonCameraSim(m_camera, simulatedCamProps);
    }

    public PhotonCameraSim getSimulatedCamera() {
        return m_cameraSim;
    }

    public Transform3d getTransform() {
        return m_robotToCam;
    }
}
