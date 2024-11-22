/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.tracking;

import java.util.List;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;

/** A generic game piece tracker interface */
public interface PieceTracker {
    default List<Vector2dU<Length>> getTargetLocationList(Pose2dU<Length> currentPose) {
        return getRobotRelativeDeltas().stream()
                .map(in -> in.rotate(currentPose.getRotation()).add(currentPose.getVector()))
                .toList();
    }

    List<Vector2dU<Length>> getRobotRelativeDeltas();

    List<Angle> getTargetPlaneAngles();
}
