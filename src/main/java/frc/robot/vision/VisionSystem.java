/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.logging.LogBuilder;
import frc.robot.pose.PoseProvider;
import frc.robot.vision.layout.CameraLayout;
import frc.robot.vision.layout.ProwlerLayout;
import frc.robot.vision.posevision.PoseVision;
import frc.robot.vision.posevision.VisionPoseResult;
import frc.robot.vision.tracking.PieceTracker;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.growingstems.frc.util.WpiTimeSource;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.RangeU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Measurements.Velocity;
import org.growingstems.util.timer.Timer;

/** Container for all robot vision systems */
public class VisionSystem {
    private CameraLayout m_layout;

    // Vision Part of Pose Estimator
    private final Timer m_visionTimer = new WpiTimeSource().createTimer();

    // TODO: Might need to recalibrate
    private static final RangeU<Length> k_visionTranslationStd =
            new RangeU<>(Length.inches(4.0), Length.inches(70.0));
    private static final RangeU<Angle> k_visionRotationStd =
            new RangeU<>(Angle.degrees(5.0), Angle.degrees(90.0));

    private static final Velocity velocityCutoff = Velocity.metersPerSecond(4.75);
    private static final AngularVelocity angVelCutoff = AngularVelocity.degreesPerSecond(120.0);

    private final Consumer<Time> m_logTimeSinceVision;

    public static class PoseVisionResults {
        public final Pose2dU<Length> visionRobotPose;
        public final Time timestamp;
        public final Matrix<N3, N1> visionMeasurementStdDevs;

        public PoseVisionResults(
                Pose2dU<Length> visionRobotPose, Time timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
            this.visionRobotPose = visionRobotPose;
            this.timestamp = timestamp;
            this.visionMeasurementStdDevs = visionMeasurementStdDevs;
        }
    }

    public VisionSystem(LogBuilder builder) {
        // TODO: Implement robot versions for 9072
        m_layout = new ProwlerLayout();
        m_visionTimer.start();

        // Logging
        m_logTimeSinceVision =
                builder.makeSyncLogEntry("Vision/Time Since Last Result", builder.timeType_s, Time.ZERO);
    }

    public PoseVision[] getPoseVisions() {
        return m_layout.getPoseVisions();
    }

    public List<PieceTracker> getTrackerVisions() {
        return m_layout.getTrackers();
    }

    private static Vector<N3> makeStdVec(Length translationStd, Angle rotationStd) {
        return VecBuilder.fill(
                translationStd.asMeters(), translationStd.asMeters(), rotationStd.asRadians());
    }

    public List<PoseVisionResults> getResults(PoseProvider poseProvider) {
        var resultsList = new ArrayList<PoseVisionResults>();

        // Exceeds velocity limiters
        if (poseProvider.getFieldVelocity().getMagnitude().gt(velocityCutoff)
                || poseProvider.getYawRate().gt(angVelCutoff)) {
            return Collections.emptyList();
        }

        Time timeSinceVision = m_visionTimer.get();
        for (var vision : getPoseVisions()) {
            var visionResults = vision.getCombinedEstimation();
            if (visionResults.isEmpty()) {
                continue;
            }
            m_visionTimer.reset();
            timeSinceVision = Time.ZERO;

            var singleResult = getSingleResult(visionResults.get(), vision.getDistanceCutoff());
            if (singleResult.isPresent()) {
                resultsList.add(singleResult.get());
            }
        }

        m_logTimeSinceVision.accept(timeSinceVision);
        return resultsList;
    }

    public Optional<PoseVisionResults> getSingleResult(
            VisionPoseResult verifiedResults, Length distanceCutoff) {
        if (verifiedResults.tagData.size() < 1) {
            System.out.println("Attempted to use MultiPNP result with no tags??");
            return Optional.empty();
        }

        // Loops over all tags used
        var aggregateAmgibuity = 0.0;
        for (var data : verifiedResults.tagData) {
            if (data.translation.getMagnitude().gt(distanceCutoff)) {
                return Optional.empty();
            }
            aggregateAmgibuity += data.ambiguity;
        }
        var avgAmbiguity = aggregateAmgibuity / verifiedResults.tagData.size();

        // TODO test: should we use

        // verifiedResults.estimationAmbiguity
        // instead?

        var lengthStd =
                k_visionTranslationStd.getWidth().mul(avgAmbiguity).add(k_visionTranslationStd.getLow());
        var rotStd = k_visionRotationStd.getWidth().mul(avgAmbiguity).add(k_visionRotationStd.getLow());
        return Optional.of(new PoseVisionResults(
                verifiedResults.getPose(), verifiedResults.timeStamp, makeStdVec(lengthStd, rotStd)));
    }

    public Vector2dU<Length> getGamePieceOfInterest(Pose2dU<Length> robotPose) {
        var notePoses = getTrackerVisions().stream()
                .flatMap(v -> v.getTargetLocationList(robotPose).stream())
                .toList();
        // Telemetry.TeleDriveTeam.field
        //         .getObject("Notes")
        //         .setPoses(notePoses.stream()
        //                 .map(v -> new Pose2d(v.getX().asMeters(), v.getY().asMeters(), new
        // Rotation2d(0.0)))
        //                 .toList());

        var closestNote = notePoses.stream()
                .min(Comparator.comparingDouble(p -> p.rangeTo(robotPose.getVector()).asInches()));

        if (closestNote.isPresent()) {
            return closestNote.get();
        } else {
            return robotPose
                    .getVector()
                    .add(Vector2dU.fromPolar(Length.feet(5.0), robotPose.getRotation()));
        }
    }
}
