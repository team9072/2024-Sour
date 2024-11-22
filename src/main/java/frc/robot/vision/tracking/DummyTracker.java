/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.tracking;

import java.util.Collections;
import java.util.List;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;

public class DummyTracker implements PieceTracker {
    @Override
    public List<Vector2dU<Length>> getRobotRelativeDeltas() {
        return Collections.emptyList();
    }

    @Override
    public List<Angle> getTargetPlaneAngles() {
        return Collections.emptyList();
    }
}
