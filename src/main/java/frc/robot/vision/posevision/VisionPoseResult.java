/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.posevision;

import java.util.List;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.Vector3dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;

public class VisionPoseResult {
    public final Vector3dU<Length> locationXYZ;
    public final Vector3dU<Angle> rotationYPR;
    public final List<TagResultData> tagData;
    public final Time timeStamp;
    public final double estimationAmbiguity;

    public VisionPoseResult(
            Vector3dU<Length> translation,
            Vector3dU<Angle> rotation,
            List<TagResultData> tagData,
            Time ts,
            double estAmbiguity) {
        rotationYPR = rotation;
        this.locationXYZ = translation;
        this.tagData = tagData;
        this.timeStamp = ts;
        this.estimationAmbiguity = estAmbiguity;
    }

    public Pose2dU<Length> getPose() {
        return new Pose2dU<>(locationXYZ.getX(), locationXYZ.getY(), rotationYPR.getZ());
    }

    public static class TagResultData {
        /** Transformation of robot-to-tag */
        public final Vector3dU<Length> translation;

        /** Transformation of robot-to-tag */
        public final Vector3dU<Angle> rotationYPR;

        public final int id;
        public final double ambiguity;

        /** Translations are robot-to-tag */
        public TagResultData(
                Vector3dU<Length> translation, Vector3dU<Angle> rotation, int id, double ambiguity) {
            rotationYPR = rotation;
            this.translation = translation;
            this.id = id;
            this.ambiguity = ambiguity;
        }
    }
}
