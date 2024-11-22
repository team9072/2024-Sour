/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.actuators;

public class Pidf {
    public double p = 0.0;
    public double i = 0.0;
    public double d = 0.0;
    public double f = 0.0;

    public Pidf(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public Pidf() {
        this(0.0, 0.0, 0.0, 0.0);
    }
}
