/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.math;

import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Unit;

public interface SpatialValley<U extends Unit<U>, Q extends Unit<Q>, E extends Unit<E>> {
    // derivative of spatial modifier function (lets call it F) w.r.t distance
    Vector2dU<E> vectorMap(Vector2dU<Q> in);

    // The partial derivatives of a distance function: Vec2d<U> -> U
    Q xPartial(Vector2dU<U> in);

    Q yPartial(Vector2dU<U> in);

    // Gradient operator on F( Dist(x, y) )
    default Vector2dU<E> grad(Vector2dU<U> pos) {
        return vectorMap(new Vector2dU<Q>(xPartial(pos), yPartial(pos))).neg();
    }
}
