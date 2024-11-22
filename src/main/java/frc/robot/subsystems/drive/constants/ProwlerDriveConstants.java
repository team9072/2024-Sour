/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.subsystems.drive.constants;

import org.growingstems.measurements.Angle;

public class ProwlerDriveConstants extends GrowlerDriveConstants {
    @Override
    protected Angle getFrontLeftOffset() {
        return new Angle(0.373291).add(new Angle(0.5)).neg().difference(Angle.ZERO);
    }

    @Override
    protected Angle getBackLeftOffset() {
        return new Angle(-0.045166)
                .add(new Angle(0.5))
                .sub(Angle.degrees(90.0))
                .neg()
                .difference(Angle.ZERO);
    }

    @Override
    protected Angle getBackRightOffset() {
        return new Angle(0.203125).add(new Angle(0.5)).neg().difference(Angle.ZERO);
    }

    @Override
    protected Angle getFrontRightOffset() {
        return new Angle(-0.044678).add(new Angle(0.5)).neg().difference(Angle.ZERO);
    }
}
