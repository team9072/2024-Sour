/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */
package frc.library.actuators;

import org.growingstems.math.RangeU;
import org.growingstems.measurements.Unit;

public interface VelocityActuator<P extends Unit<P>, V extends Unit<V>>
        extends PositionActuator<P> {
    RangeU<V> getVelocityRange();

    boolean setVelocityClosedLoop(V velocity);

    boolean setVelocityClosedLoop(V velocity, double feedforward);

    V getVelocity();

    V getGoalVelocity();

    V getVelocityError();

    void setAcceleration(V acceleration);

    void setMaxVelocity(V maxVelocity);
}
