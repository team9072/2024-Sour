/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.auto.gschoreo;

import com.choreo.lib.ChoreoTrajectoryState;
import java.util.List;

public class TrajectoryDataStruct {
    public final List<GsChorEvent> eventMarkers;
    public final List<ChoreoTrajectoryState> samples;

    public TrajectoryDataStruct() {
        eventMarkers = List.of();
        samples = List.of();
    }

    public TrajectoryDataStruct(List<GsChorEvent> eventMarkers, List<ChoreoTrajectoryState> samples) {
        this.eventMarkers = eventMarkers;
        this.samples = samples;
    }
}
