/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import org.growingstems.math.RangeU;
import org.growingstems.measurements.Unit;

public interface PositionActuator<P extends Unit<P>> extends MotorActuator {
    RangeU<P> getPositionalRange();

    void calibrateOffset(P physicalPosition);

    void setReverseSoftLimitThreshold_su(double softLimit_su);

    void setForwardSoftLimitThreshold_su(double softLimit_su);

    void enableReverseSoftLimit(boolean enable);

    void enableForwardSoftLimit(boolean enable);

    boolean setPositionClosedLoop(P position);

    boolean setPositionClosedLoop(P position, double feedforward);

    P getPosition();

    P getGoalPosition();

    P getPositionError();
}
