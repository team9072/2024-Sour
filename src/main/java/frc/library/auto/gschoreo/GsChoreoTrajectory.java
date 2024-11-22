/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.auto.gschoreo;

import com.choreo.lib.ChoreoTrajectory;
import java.util.Collections;
import java.util.Set;
import org.growingstems.measurements.Measurements.Time;

public class GsChoreoTrajectory {
    private final Set<GsChorEvent> m_events;
    private final ChoreoTrajectory m_traj;
    private final Time m_timeBuffer;

    public GsChoreoTrajectory() {
        m_traj = new ChoreoTrajectory();
        m_events = Collections.emptySet();
        m_timeBuffer = Time.ZERO;
    }

    public GsChoreoTrajectory(ChoreoTrajectory traj, Set<GsChorEvent> events, Time timeBuffer) {
        m_traj = traj;
        m_events = events;
        m_timeBuffer = timeBuffer;
    }

    public GsChoreoTrajectory(ChoreoTrajectory traj, Set<GsChorEvent> events) {
        m_traj = traj;
        m_events = events;
        m_timeBuffer = Time.ZERO;
    }

    public GsChoreoTrajectory flip() {
        return new GsChoreoTrajectory(m_traj.flipped(), m_events, m_timeBuffer);
    }

    public ChoreoTrajectory getTrajectory() {
        return m_traj;
    }

    public final Set<GsChorEvent> getEventList() {
        return m_events;
    }

    public final Time getTimeBuffer() {
        return m_timeBuffer;
    }
}
