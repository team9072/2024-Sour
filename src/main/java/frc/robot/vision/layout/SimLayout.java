/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.layout;

import frc.robot.vision.configs.CameraConfig;
import frc.robot.vision.posevision.PhotonPoseSim;
import frc.robot.vision.posevision.PoseVision;
import frc.robot.vision.tracking.DummyTracker;
import frc.robot.vision.tracking.PieceTracker;
import java.util.List;
import org.photonvision.simulation.SimCameraProperties;

public class SimLayout extends GrowlerLayout {

    @Override
    protected void init(List<CameraConfig> visionConfigs, List<CameraConfig> trackerConfigs) {
        m_poseVisions = visionConfigs.stream()
                .map(this::generateVision)
                .toList()
                .toArray(new PoseVision[visionConfigs.size()]);

        m_trackers = trackerConfigs.stream().map(this::generateTracker).toList();
    }

    @Override
    protected PoseVision generateVision(CameraConfig configs) {
        // TODO this can actually be populated with real values maybe
        SimCameraProperties cameraProp = new SimCameraProperties();

        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        return new PhotonPoseSim(
                configs.name,
                cameraProp,
                configs.getTransform(),
                configs.getYPR(),
                configs.rejectionDistance);
    }

    @Override
    protected PieceTracker generateTracker(CameraConfig configs) {
        // TODO maybe simulate trackers too idk
        return new DummyTracker();
        // return new PhotonTracker(configs.name, configs.getTransform(), configs.getYPR());
    }
}
