/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.layout;

import frc.robot.vision.configs.CameraConfig;
import frc.robot.vision.posevision.PhotonPoseVision;
import frc.robot.vision.posevision.PoseVision;
import frc.robot.vision.tracking.PhotonTracker;
import frc.robot.vision.tracking.PieceTracker;
import java.util.List;

/**
 * Enables the construction of camera layouts from provided camera configurations, implemented for
 * each robot
 */
public abstract class CameraLayout {
    protected PoseVision[] m_poseVisions;
    protected List<PieceTracker> m_trackers;

    public PoseVision[] getPoseVisions() {
        return m_poseVisions;
    }

    public List<PieceTracker> getTrackers() {
        return m_trackers;
    }

    protected void init(List<CameraConfig> visionConfigs, List<CameraConfig> trackerConfigs) {
        m_poseVisions = visionConfigs.stream()
                .map(this::generateVision)
                .toList()
                .toArray(new PoseVision[visionConfigs.size()]);

        m_trackers = trackerConfigs.stream().map(this::generateTracker).toList();
    }

    protected PoseVision generateVision(CameraConfig configs) {
        return new PhotonPoseVision(
                configs.name, configs.getTransform(), configs.getYPR(), configs.rejectionDistance);
    }

    protected PieceTracker generateTracker(CameraConfig configs) {
        return new PhotonTracker(configs.name, configs.getTransform(), configs.getYPR());
    }
}
