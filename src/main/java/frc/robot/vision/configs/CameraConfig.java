/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.configs;

import java.util.Optional;
import org.growingstems.math.Vector3dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;

/**
 * A utility class containing all relevant information that could be used to construct high-level
 * camera systems
 */
public class CameraConfig {
    public String name;

    public Length cameraX;
    public Length cameraY;
    public Length cameraZ;

    public Angle cameraYaw;
    public Angle cameraPitch;
    public Angle cameraRoll;

    public Optional<CameraIntrinsics> intrinsics;

    public Length rejectionDistance;

    public CameraConfig(
            String name,
            Length cameraX,
            Length cameraY,
            Length cameraZ,
            Angle cameraYaw,
            Angle cameraPitch,
            Angle cameraRoll,
            Length rejection,
            Optional<CameraIntrinsics> intrinsics) {
        this.name = name;
        this.cameraX = cameraX;
        this.cameraY = cameraY;
        this.cameraZ = cameraZ;
        this.cameraYaw = cameraYaw;
        this.cameraPitch = cameraPitch;
        this.cameraRoll = cameraRoll;
        this.rejectionDistance = rejection;

        this.intrinsics = intrinsics;
    }

    public Vector3dU<Length> getTransform() {
        return new Vector3dU<Length>(cameraX, cameraY, cameraZ);
    }

    public Vector3dU<Angle> getYPR() {
        return new Vector3dU<Angle>(cameraYaw, cameraPitch, cameraRoll);
    }

    public class CameraIntrinsics {
        public final Angle verticalFov;
        public final Angle horizontalFov;
        public final int horizontalRes;
        public final int verticalRes;

        public CameraIntrinsics(
                Angle verticalFov, Angle horizontalFov, int horizontalRes, int verticalRes) {
            this.verticalFov = verticalFov;
            this.horizontalFov = horizontalFov;
            this.horizontalRes = horizontalRes;
            this.verticalRes = verticalRes;
        }
    }
}
