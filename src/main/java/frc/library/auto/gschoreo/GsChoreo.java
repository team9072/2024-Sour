/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.auto.gschoreo;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonDeserializationContext;
import com.google.gson.JsonDeserializer;
import com.google.gson.JsonElement;
import com.google.gson.JsonParseException;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.library.algorithms.BipartiteCollapse;
import frc.library.auto.SerializedCommand;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.apache.commons.math3.util.Pair;
import org.growingstems.frc.util.WpiTimeSource;
import org.growingstems.math.Pose2dU;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;

public class GsChoreo {
    private static final class EventDeserializer implements JsonDeserializer<GsChorEvent> {
        @Override
        public GsChorEvent deserialize(
                JsonElement json, Type typeOfT, JsonDeserializationContext context)
                throws JsonParseException {
            var jsonObj = json.getAsJsonObject();
            var time = Time.seconds(jsonObj.get("timestamp").getAsDouble());
            Supplier<Command> command =
                    () -> SerializedCommand.parseCommand(jsonObj.get("command").getAsJsonObject());
            return new GsChorEvent(time, command);
        }
    }

    private static final Gson gson;

    static {
        var builder = new GsonBuilder();
        builder.registerTypeAdapter(GsChorEvent.class, new EventDeserializer());
        gson = builder.create();
    }

    /** Create a command to follow a Choreo path. */
    public static Command choreoSwerveCommand(
            GsChoreoTrajectory trajectory,
            Supplier<Pose2dU<Length>> poseSupplier,
            BiFunction<Pose2dU<Length>, ChoreoTrajectoryState, ChassisSpeeds> controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Subsystem... requirements) {
        var timer = new WpiTimeSource().createTimer();

        var queue = trajectory.getEventList();

        // Form a bipartite relation between events and required subsystems
        var eventRelation = BipartiteCollapse.collapse(queue.stream()
                .map(in -> new Pair<>(in, in.getCommand().getRequirements()))
                .toList());

        // Turn conflicting commands into sequential operations that depend on a time trigger
        var coordinatedCommands = eventRelation.stream()
                .map(in -> {
                    var commandList = new ArrayList<>(in.getFirst().stream().toList());
                    // Comparison reversed to get lowest timestamps first
                    commandList.sort(Comparator.comparingDouble((t) -> t.getTimestamp().asSeconds()));
                    var group = new SequentialCommandGroup();
                    commandList.stream().forEachOrdered(c -> {
                        var cmd = new WaitUntilCommand(() -> timer.get().ge(c.getTimestamp()))
                                .andThen(c.getCommand());
                        group.addCommands(cmd);
                    });
                    return group;
                })
                .toArray(Command[]::new);

        return (new FunctionalCommand(
                                () -> {
                                    timer.start();
                                    timer.reset();
                                },
                                () -> {
                                    outputChassisSpeeds.accept(controller.apply(
                                            poseSupplier.get(),
                                            trajectory.getTrajectory().sample(timer.get().asSeconds())));
                                },
                                interrupted -> {
                                    timer.stop();
                                    if (interrupted) {
                                        outputChassisSpeeds.accept(new ChassisSpeeds());
                                    } else {
                                        outputChassisSpeeds.accept(
                                                trajectory.getTrajectory().getFinalState().getChassisSpeeds());
                                    }
                                },
                                () -> timer.hasElapsed(Time.seconds(trajectory.getTrajectory().getTotalTime())),
                                requirements)
                        .andThen(new WaitCommand(trajectory.getTimeBuffer().asSeconds())))
                .deadlineWith(coordinatedCommands);
    }

    public static BiFunction<Pose2dU<Length>, ChoreoTrajectoryState, ChassisSpeeds>
            GSChoreoController(
                    PIDController xController, PIDController yController, PIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double rotationFF = referenceState.angularVelocity;

            double xFeedback = xController.calculate(pose.getX().asMeters(), referenceState.x);
            double yFeedback = yController.calculate(pose.getY().asMeters(), referenceState.y);
            double rotationFeedback =
                    rotationController.calculate(pose.getRotation().asRadians(), referenceState.heading);

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback,
                    yFF + yFeedback,
                    rotationFF + rotationFeedback,
                    new Rotation2d(pose.getRotation().asRadians()));
        };
    }

    public static GsChoreoTrajectory getTrajectory(String trajName) {
        return getTrajectory(trajName, Time.ZERO);
    }

    public static GsChoreoTrajectory getTrajectory(String trajName, Time timebuffer) {
        var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        var traj_file = new File(traj_dir, trajName + ".traj");

        return loadFile(traj_file, timebuffer);
    }

    private static GsChoreoTrajectory loadFile(File path, Time time) {
        try {
            var reader = new BufferedReader(new FileReader(path));
            TrajectoryDataStruct traj = gson.fromJson(reader, TrajectoryDataStruct.class);

            return new GsChoreoTrajectory(
                    new ChoreoTrajectory(traj.samples), new HashSet<>(traj.eventMarkers), time);
        } catch (Exception ex) {
            DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
        }
        return null;
    }
}
