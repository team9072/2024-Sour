/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.layout;

import frc.robot.vision.configs.CameraConfig;
import java.util.List;

public class ProwlerLayout extends GrowlerLayout {
    @Override
    protected List<CameraConfig> getVisionConfigs() {
        m_rightShooterVisionCam.name = "GS7";
        return List.of(m_rightShooterVisionCam);
    }

    @Override
    protected List<CameraConfig> getTrackerConfigs() {
        return List.of();
    }
}
