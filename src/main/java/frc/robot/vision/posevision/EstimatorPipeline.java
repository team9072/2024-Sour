/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.posevision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.library.utils.ConversionUtils;
import frc.robot.vision.posevision.VisionPoseResult.TagResultData;
import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;
import org.growingstems.measurements.Measurements.Time;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class EstimatorPipeline {
    private final AprilTagFieldLayout m_fieldTags;
    private final PhotonCamera m_camera;
    private Transform3d m_robotToCamera;

    private double m_cachedTimestamp = -1;

    public EstimatorPipeline(
            AprilTagFieldLayout fieldTags, PhotonCamera camera, Transform3d robotToCamera) {
        m_fieldTags = fieldTags;
        m_camera = camera;
        m_robotToCamera = robotToCamera;
    }

    public Optional<VisionPoseResult> update() {
        var camResult = m_camera.getLatestResult();

        var timestamp = camResult.getTimestampSeconds();

        // Check for valid timestamp and that targets exist
        if (timestamp < 0
                || m_cachedTimestamp > 0 && Math.abs(m_cachedTimestamp - timestamp) < 1e-6
                || !camResult.hasTargets()) {
            return Optional.empty();
        }

        if (camResult.getMultiTagResult().estimatedPose.isPresent) {
            // TODO here we can actually select best based off of what we wanna do
            var pnpResult = camResult.getMultiTagResult();
            var best_tf = pnpResult.estimatedPose.best;

            var best = new Pose3d()
                    .plus(best_tf) // field-to-camera
                    .relativeTo(m_fieldTags.getOrigin())
                    .plus(m_robotToCamera.inverse()); // field-to-robot

            ArrayList<TagResultData> tagData = new ArrayList<TagResultData>();

            for (PhotonTrackedTarget target : camResult.getTargets()) {
                var tagId = target.getFiducialId();
                // 2024 ids
                if (tagId < 1 || tagId > 16) {
                    System.err.println(
                            "WARNING: Invalid tag detected in MultiTag approximation. THIS SHOULD NEVER HAPPEN");
                    return Optional.empty();
                }

                // If we are auto, ignore source tags
                if ((DriverStation.isAutonomousEnabled() || DriverStation.isDisabled())
                        && Set.of(1, 2, 9, 10).contains(tagId)) {
                    return Optional.empty();
                }

                var transform = m_robotToCamera.plus(target.getBestCameraToTarget());
                var resultData = new TagResultData(
                        ConversionUtils.fromWpi(transform.getTranslation()),
                        ConversionUtils.fromWpi(transform.getRotation()),
                        tagId,
                        target.getPoseAmbiguity());

                tagData.add(resultData);
            }

            return Optional.of(new VisionPoseResult(
                    ConversionUtils.fromWpi(best.getTranslation()),
                    ConversionUtils.fromWpi(best.getRotation()),
                    tagData,
                    Time.seconds(timestamp),
                    pnpResult.estimatedPose.ambiguity));
        }

        return Optional.empty();
    }
}
