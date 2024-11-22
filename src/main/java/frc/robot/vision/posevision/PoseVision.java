/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.posevision;

import java.util.Optional;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;

/** A generic vision-based localizer interface */
public interface PoseVision {
    Optional<VisionPoseResult> getCombinedEstimation();

    Time getLatency();

    boolean isConnected();

    // Distance of 20ft by default
    default Length getDistanceCutoff() {
        return Length.feet(20.0);
    }
}
