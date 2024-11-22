/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.posevision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.growingstems.math.Vector3dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;
import org.photonvision.PhotonCamera;

public class PhotonPoseVision implements PoseVision {
    protected PhotonCamera m_camera;
    private EstimatorPipeline m_poseEstimator;

    protected final Transform3d m_robotToCam;

    protected final Length m_rejectionDistance;

    public PhotonPoseVision(
            String camera,
            Vector3dU<Length> cameraPosition,
            Vector3dU<Angle> cameraYawPitchRoll,
            Length rejectionDistance) {
        m_robotToCam = new Transform3d(
                cameraPosition.getX().asMeters(),
                cameraPosition.getY().asMeters(),
                cameraPosition.getZ().asMeters(),
                new Rotation3d(
                        cameraYawPitchRoll.getZ().asRadians(),
                        cameraYawPitchRoll.getY().asRadians(),
                        cameraYawPitchRoll.getX().asRadians()));
        var atfl = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        m_camera = new PhotonCamera(camera);
        m_poseEstimator = new EstimatorPipeline(atfl, m_camera, m_robotToCam);
        m_rejectionDistance = rejectionDistance;
    }

    @Override
    public Length getDistanceCutoff() {
        return m_rejectionDistance;
    }

    @Override
    public Time getLatency() {
        var result = m_camera.getLatestResult();
        return Time.milliseconds(result.getLatencyMillis());
    }

    @Override
    public Optional<VisionPoseResult> getCombinedEstimation() {
        return m_poseEstimator.update();
    }

    @Override
    public boolean isConnected() {
        return m_camera.isConnected();
    }
}
