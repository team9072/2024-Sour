/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.layout;

import java.util.Collections;

public class DummyLayout extends CameraLayout {

    // Dummy is honestly less useful now because everything can be treated as
    // an array, which can be empty. I think this is still reasonable at the
    // moment though - will

    public DummyLayout() {
        init(Collections.emptyList(), Collections.emptyList());
    }
}
