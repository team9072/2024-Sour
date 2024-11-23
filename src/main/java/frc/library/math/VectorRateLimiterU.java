/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Measurements.DivByTime;
import org.growingstems.measurements.Measurements.MulByTime;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Unit;
import org.growingstems.signals.api.SignalModifier;
import org.growingstems.util.timer.TimeSource;
import org.growingstems.util.timer.Timer;

public class VectorRateLimiterU<U extends Unit<U> & DivByTime<V>, V extends Unit<V> & MulByTime<U>>
        implements SignalModifier<Vector2dU<U>, Vector2dU<U>> {
    private Vector2dU<U> m_pastPosition;
    private final Timer m_timer;
    private V m_rate;

    public VectorRateLimiterU(V rate, TimeSource timeSource) {
        U zero = rate.mul(Time.ZERO);
        m_pastPosition = new Vector2dU<U>(zero, zero);

        m_timer = timeSource.createTimer();
        m_timer.start();

        m_rate = rate;
    }

    public void setRate(V rate) {
        m_rate = rate;
    }

    public Vector2dU<U> update(Vector2dU<U> input) {
        if (m_rate.isInfinite() || m_rate.isNaN()) {
            m_pastPosition = input;
            return input;
        }

        Vector2dU<U> diff = input.sub(m_pastPosition);
        var maxDiff = m_rate.mul(m_timer.reset());

        if (diff.getMagnitude().gt(maxDiff)) {
            return m_pastPosition = m_pastPosition.add(Vector2dU.fromPolar(maxDiff, diff.getAngle()));
        } else {
            return m_pastPosition = input;
        }
    }
}
