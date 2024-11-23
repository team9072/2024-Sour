/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.constants;

import frc.robot.AllianceUtils;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;

public class FieldDimensions {
    // TODO cite the location in the rulebook when getting values

    // pg 21/149 section 5 ARENA
    public static final Length k_fieldWidth = Length.inches(323.25);
    public static final Length k_fieldLength = Length.inches(651.25);

    public static final Length k_centerX = k_fieldLength.div(2.0);
    public static final Length k_centerY = k_fieldWidth.div(2.0);

    // pg 22/149 section 5 ARENA
    public static final Length k_guardrailHeight = Length.inches(20.0);

    public static final Length k_gateLength = Length.inches(38.0);

    // pg 23/149 section 5 ARENA
    public static final Length k_tapeLength = Length.inches(2.0);

    // pg 24/149 section 5 ARENA
    public static final Length k_allianceLength = Length.inches(118.25);
    public static final Length k_allianceWidth = Length.inches(323.125);

    public static final Length k_ampZoneLength = Length.inches(130.0);
    public static final Length k_ampZoneWidth = Length.inches(17.75);

    public static final Length k_robotStartingZoneLength = Length.inches(284.125);
    public static final Length k_robotStartingZoneWidth = Length.inches(76.125);

    public static final Length k_sourceAreaLength = Length.inches(190.5);
    public static final Length k_sourceAreaWidth = Length.inches(60.75);

    // pg 25/149 section 5 ARENA
    public static final Length k_ampHoleLength = Length.inches(3.875);
    public static final Length k_ampHoleWidth = Length.inches(24.0);
    public static final Length k_ampHoleHeight = Length.inches(18.0);

    // pg 26/149 section 5 ARENA
    public static final Length k_ampAllianceDistance = Length.inches(49.5);

    public static final Length k_sourceHoleLength = Length.inches(6.0);
    public static final Length k_sourceHoleWidth = Length.inches(75.25);
    public static final Length k_sourceHoleHeight = Length.inches(36.75);

    // pg 27/149 section 5 ARENA
    public static final Length k_stageAllianceDistance = Length.inches(121.0);

    // pg 28/149 section 5 ARENA
    public static final Length k_trapFloorHeight = Length.inches(56.5);

    // pg 29/149 section 5 ARENA
    public static final Length k_microphoneHeight = Length.inches(12.0);
    public static final Length k_microphoneRadius = Length.inches(1.25);
    public static final Length k_microphoneFloorHeight = Length.inches(76.25);

    // pg 31/149 section 5 ARENA
    public static final Length k_speakerLowFloorHeight = Length.inches(78.0);
    public static final Length k_speakerHighFloorHeight = Length.inches(82.875);
    public static final Length k_speakerOpeningWidth = Length.inches(41.375);
    public static final Length k_speakerOverhang = Length.feet(1.0).add(Length.inches(6.0));
    public static final Angle k_speakerOpeningAngle = Angle.degrees(14.0);

    // pg 32/149 section 5 ARENA
    public static final Length k_subwooferHeight = Length.inches(37.0);
    public static final Length k_subwooferWidth = Length.inches(36.125);

    public static final Length k_driverStationLength = Length.inches(12.25);
    public static final Length k_driverStationWidth = Length.inches(69.0);
    public static final Length k_driverStationHeight = Length.inches(78.75);

    // pg 34/149 section 5 ARENA
    public static final Length k_noteInsideDiameter = Length.inches(10.0);
    public static final Length k_noteOutsideDiameter = Length.inches(14.0);
    public static final Length k_noteThickness = Length.inches(2.0);

    // pg 35/149 section 5 ARENA
    public static final Length k_aprilTagLength = Length.inches(8.125);
    public static final Length k_apriltagWidth = Length.inches(8.125);

    // pg 37/149 section 5 ARENA
    public static final Length k_sourceAprilTagFloorHeight = Length.inches(48.125);
    public static final Length k_sourceAprilTagWidthFromCenter = Length.inches(19.375);

    public static final Length k_speakerAprilTagFloorHeight = Length.inches(45.875);
    public static final Length k_speakerAprilTagWidthFromCenter = Length.inches(17.0);

    // pg 38/149 section 5 ARENA
    public static final Length k_ampAprilTagFloorHeight = Length.inches(48.125);

    // pg 39/149 section 5 ARENA
    public static final Length k_stageAprilTagFloorHieght = Length.inches(47.5);
    public static final Length k_stageAprilTagBehindMaterial = Length.inches(0.25);

    // pg 45 6.3.4 Figure 6-2
    public static final Length k_closeNoteWallOffset = Length.feet(9).add(Length.inches(6)); // 9'6"
    public static final Length k_closeNoteVerticalSpacing =
            Length.feet(4).add(Length.inches(9)); // 4'9"

    public static final Length k_midlineNoteVerticalSpacing =
            Length.feet(5).add(Length.inches(6)); // 5'6"\

    /** See {@link StagePillar} for details on these values */
    public static final Length k_stageCenterPillarX = Length.inches(133.0);

    public static final Length k_stageSideLength = Length.inches(104.26);
    public static final Length k_stageXOffset =
            k_stageSideLength.mul(Angle.degrees(30.0).cos());

    public static final Vector2dU<Length> k_blueSpeakerXYTarget =
            new Vector2dU<Length>(k_speakerOverhang.div(2.0), k_centerY.add(k_closeNoteVerticalSpacing));

    public static final Vector2dU<Length> k_blueLoftXYTarget = new Vector2dU<Length>(
            Length.ZERO, k_centerY.add(k_closeNoteVerticalSpacing).add(Length.feet(8.0)));

    public static Vector2dU<Length> getSpeakerLocation(boolean isRed) {
        return isRed
                ? AllianceUtils.transform(FieldDimensions.k_blueSpeakerXYTarget)
                : FieldDimensions.k_blueSpeakerXYTarget;
    }

    public static Vector2dU<Length> getLoftLocation(boolean isRed) {
        return isRed
                ? AllianceUtils.transform(FieldDimensions.k_blueLoftXYTarget)
                : FieldDimensions.k_blueLoftXYTarget;
    }
}
