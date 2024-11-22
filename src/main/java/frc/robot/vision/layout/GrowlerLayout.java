/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision.layout;

import frc.robot.vision.configs.CameraConfig;
import java.util.List;
import java.util.Optional;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;

public class GrowlerLayout extends CameraLayout {
    // Shooter-Side Camera Configuration
    protected static final Length k_shooterVisionX = Length.inches(-12.0); // Positive towards intake
    protected static final Length k_shooterVisionY =
            Length.inches(11.0); // Positive is to robot left (shooter right)
    protected static final Length k_shooterVisionZ = Length.inches(8.5); // Always positive
    protected static final Angle k_shooterVisionPitch =
            Angle.degrees(-22.0); // Negative is above ground
    protected static final Angle k_shooterVisionRoll = Angle.ZERO;
    // Often needs retuned
    protected static final Angle k_leftShooterVisionYaw =
            Angle.degrees(-4.0).difference(Angle.PI); // Zero is towards intake, Left-Positive
    // Often needs retuned
    protected static final Angle k_rightShooterVisionYaw =
            Angle.degrees(4.0).difference(Angle.PI); // Zero is towards intake, Left-Positive

    // Intake-Side Camera Configuration
    protected static final Length k_frontVisionX =
            Length.inches(29.75).div(2.0).sub(Length.inches(8.75)); // Positive towards intake
    protected static final Length k_frontVisionY =
            Length.inches(25.5).div(2.0); // Positive is to robot left (intake left)
    protected static final Length k_frontVisionZ = Length.inches(22.25); // Always positive
    protected static final Angle k_frontVisionPitch =
            Angle.degrees(-14.5); // Negative is above ground
    protected static final Angle k_frontVisionRoll = Angle.ZERO;
    // Often needs retuned
    protected static final Angle k_frontLeftVisionYaw =
            Angle.degrees(7.62 - 2.5); // Zero is towards intake, Left-Positive
    // Often needs retuned
    protected static final Angle k_frontRightVisionYaw =
            Angle.degrees(-7.62 + 2.5 - 3.5); // Zero is towards intake, Left-Positive

    protected static final Length k_shooterDistanceCutoff = Length.feet(20.0);
    protected static final Length k_sourceDistanceCutoff = Length.feet(15.0);

    protected final CameraConfig m_leftShooterVisionCam = new CameraConfig(
            "GS9",
            k_shooterVisionX,
            k_shooterVisionY.neg(),
            k_shooterVisionZ,
            k_leftShooterVisionYaw,
            k_shooterVisionPitch,
            k_shooterVisionRoll,
            k_shooterDistanceCutoff,
            Optional.empty());

    protected final CameraConfig m_rightShooterVisionCam = new CameraConfig(
            "GS6",
            k_shooterVisionX,
            k_shooterVisionY,
            k_shooterVisionZ,
            k_rightShooterVisionYaw,
            k_shooterVisionPitch,
            k_shooterVisionRoll,
            k_shooterDistanceCutoff,
            Optional.empty());

    protected final CameraConfig m_frontLeftVisionCam = new CameraConfig(
            "GS2",
            k_frontVisionX,
            k_frontVisionY,
            k_frontVisionZ,
            k_frontLeftVisionYaw,
            k_frontVisionPitch,
            k_frontVisionRoll,
            k_sourceDistanceCutoff,
            Optional.empty());

    protected final CameraConfig m_frontRightVisionCam = new CameraConfig(
            "GS7",
            k_frontVisionX,
            k_frontVisionY.neg(),
            k_frontVisionZ,
            k_frontRightVisionYaw,
            k_frontVisionPitch,
            k_frontVisionRoll,
            k_sourceDistanceCutoff,
            Optional.empty());

    public GrowlerLayout() {
        init(getVisionConfigs(), getTrackerConfigs());
    }

    protected List<CameraConfig> getVisionConfigs() {
        return List.of(
                m_leftShooterVisionCam,
                m_rightShooterVisionCam,
                m_frontLeftVisionCam,
                m_frontRightVisionCam);
    }

    protected List<CameraConfig> getTrackerConfigs() {
        return List.of();
    }
}
