/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import org.growingstems.math.Vector2d;
import org.growingstems.measurements.Measurements.Frequency;
import org.growingstems.signals.api.SignalModifier;
import org.growingstems.util.timer.TimeSource;
import org.growingstems.util.timer.Timer;

public class VectorRateLimiter implements SignalModifier<Vector2d, Vector2d> {
    private Vector2d m_pastPosition = new Vector2d(0, 0);
    private final Timer m_timer;
    private Frequency m_rate;

    public VectorRateLimiter(Frequency rate, TimeSource timeSource) {
        m_timer = timeSource.createTimer();
        m_timer.start();
        m_rate = rate;
    }

    public void setRate(Frequency rate) {
        this.m_rate = rate;
    }

    public Vector2d update(Vector2d input) {
        if (m_rate.isInfinite() || m_rate.isNaN()) {
            m_pastPosition = input;
            return input;
        }

        Vector2d diff = input.sub(m_pastPosition);
        double dist = m_rate.mul(m_timer.reset()).asNone();

        if (diff.getMagnitude() > dist) {
            return m_pastPosition = m_pastPosition.add(new Vector2d(dist, diff.getAngle()));
        } else {
            return m_pastPosition = input;
        }
    }
}
