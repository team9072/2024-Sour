/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

import org.growingstems.measurements.Measurements.Frequency;
import org.growingstems.measurements.Measurements.Time;

public class AngularPidf {
    public double p = 0.0;
    public Frequency i = Frequency.ZERO;
    public Time d = Time.ZERO;

    public AngularPidf withP(double p) {
        this.p = p;
        return this;
    }

    public AngularPidf withI(Frequency i) {
        this.i = i;
        return this;
    }

    public AngularPidf withD(Time d) {
        this.d = d;
        return this;
    }
}
