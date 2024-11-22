/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library.auto;

import com.google.gson.JsonObject;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SerializedCommand {
    public static Command wrapCommand(Command command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished,
                command.getRequirements().toArray(Subsystem[]::new));
    }

    public static Command parseCommand(JsonObject json) {
        var type = json.get("type").getAsString();
        var data = json.get("data").getAsJsonObject();
        switch (type) {
            case "wait":
                return parseWaitCommand(data);
            case "named":
                return parseNamedCommand(data);
            case "sequential":
                return parseSequentialGroup(data);
            case "parallel":
                return parseParallelGroup(data);
            case "race":
                return parseRaceGroup(data);
            case "deadline":
                System.out.println("WARNING: Unsuported use of deadline command groups");
            default:
                System.out.println("WARNING: Unknown command format");
        }

        return new InstantCommand();
    }

    public static Command parseWaitCommand(JsonObject json) {
        var time = json.get("waitTime").getAsDouble();
        return new WaitCommand(time);
    }

    public static Command parseNamedCommand(JsonObject json) {
        var name = json.get("name").getAsString();
        return MarkerNameMap.getCommand(name).get();
    }

    public static Command parseSequentialGroup(JsonObject json) {
        var group = new SequentialCommandGroup();
        for (var jsonData : json.get("commands").getAsJsonArray()) {
            group.addCommands(parseCommand(jsonData.getAsJsonObject()));
        }
        return group;
    }

    public static Command parseParallelGroup(JsonObject json) {
        var group = new ParallelCommandGroup();
        for (var jsonData : json.get("commands").getAsJsonArray()) {
            group.addCommands(parseCommand(jsonData.getAsJsonObject()));
        }
        return group;
    }

    public static Command parseRaceGroup(JsonObject json) {
        var group = new ParallelRaceGroup();
        for (var jsonData : json.get("commands").getAsJsonArray()) {
            group.addCommands(parseCommand(jsonData.getAsJsonObject()));
        }
        return group;
    }
}
