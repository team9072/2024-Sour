/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.logging;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.wpilibj.RobotController;
import java.io.File;
import org.growingstems.frc.util.WpiTimeSource;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Acceleration;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Current;
import org.growingstems.measurements.Measurements.Energy;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Power;
import org.growingstems.measurements.Measurements.Temperature;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Measurements.Velocity;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.util.Timestamped;
import org.growingstems.util.logger.LogEntryType;
import org.growingstems.util.logger.LogFileBuilder;

public class LogBuilder extends LogFileBuilder {
    // Helpful initializers
    // TODO: Move this out into library
    public final Vector2dU<Velocity> zeroVelocityVector =
            new Vector2dU<>(Velocity.ZERO, Velocity.ZERO);
    public final Pose2dU<Length> zeroLengthPose = new Pose2dU<>(Length.ZERO, Length.ZERO, Angle.ZERO);

    public final LogEntryType<Time> timeType_s;
    public final LogEntryType<Time> timeType_ms;
    public final LogEntryType<Length> lengthType_in;
    public final LogEntryType<Velocity> velocityType_inps;
    public final LogEntryType<Velocity> velocityType_ftps;
    public final LogEntryType<Acceleration> accelerationType_inps2;
    public final LogEntryType<Angle> angleType_deg;
    public final LogEntryType<AngularVelocity> angularVelocityType_rpm;
    public final LogEntryType<Voltage> voltageType_volts;
    public final LogEntryType<Current> currentType_amps;
    public final LogEntryType<Energy> energyType_joules;
    public final LogEntryType<Temperature> temperatureType_celsius;
    public final LogEntryType<Power> powerType_watts;
    public final LogEntryType<Vector2dU<Length>> vector2dUType_cartesian_in;
    public final LogEntryType<Vector2dU<Velocity>> vector2dUType_cartesian_inps;
    public final LogEntryType<Vector2dU<Velocity>> vector2dUType_cartesian_ftps;
    public final LogEntryType<Vector2dU<Velocity>> vector2dUType_polar_ftps;
    public final LogEntryType<Vector2dU<Acceleration>> vector2dUType_cartesian_inps2;
    public final LogEntryType<Pose2dU<Length>> pose2dUType_in;
    public final LogEntryType<Timestamped<Pose2dU<Length>>> timestampedPose2dUType_in;
    public final LogEntryType<CANBusStatus> canBusStatusType;

    private LogBuilder(File logDir, String initialLogFileName) {
        super(logDir, initialLogFileName, "2025 Reefscape Log File", new WpiTimeSource());

        // Unit Types
        timeType_s = this.<Time>buildGroupType("Time")
                .addMember("seconds", this.doubleType, Time::asSeconds)
                .register();
        timeType_ms = this.<Time>buildGroupType("Time")
                .addMember("milliseconds", this.doubleType, Time::asMilliseconds)
                .register();
        lengthType_in = this.<Length>buildGroupType("Length")
                .addMember("inches", this.doubleType, Length::asInches)
                .register();
        velocityType_inps = this.<Velocity>buildGroupType("Velocity")
                .addMember("inches per second", this.doubleType, Velocity::asInchesPerSecond)
                .register();
        velocityType_ftps = this.<Velocity>buildGroupType("Velocity")
                .addMember("feet per second", this.doubleType, Velocity::asFeetPerSecond)
                .register();
        accelerationType_inps2 = this.<Acceleration>buildGroupType("Acceleration")
                .addMember(
                        "inches per second squared", this.doubleType, Acceleration::asInchesPerSecondSquared)
                .register();
        angleType_deg = this.<Angle>buildGroupType("Angle")
                .addMember("degrees", this.doubleType, Angle::asDegrees)
                .register();
        angularVelocityType_rpm = this.<AngularVelocity>buildGroupType("AngularVelocity")
                .addMember("rpm", this.doubleType, AngularVelocity::asRevolutionsPerMinute)
                .register();
        voltageType_volts = this.<Voltage>buildGroupType("Voltage")
                .addMember("volts", this.doubleType, Voltage::asVolts)
                .register();
        currentType_amps = this.<Current>buildGroupType("Current")
                .addMember("amps", this.doubleType, Current::asAmps)
                .register();
        energyType_joules = this.<Energy>buildGroupType("Energy")
                .addMember("joules", this.doubleType, Energy::asJoules)
                .register();
        temperatureType_celsius = this.<Temperature>buildGroupType("Temperature")
                .addMember("celsius", this.doubleType, Temperature::asCelsius)
                .register();
        powerType_watts = this.<Power>buildGroupType("Power")
                .addMember("watts", this.doubleType, Power::asWatts)
                .register();

        // Pose Types

        vector2dUType_cartesian_in = this.<Vector2dU<Length>>buildGroupType("Vector2dU")
                .addMember("X", lengthType_in, Vector2dU::getX)
                .addMember("Y", lengthType_in, Vector2dU::getY)
                .register();
        vector2dUType_cartesian_inps = this.<Vector2dU<Velocity>>buildGroupType("Vector2dU")
                .addMember("X", velocityType_inps, Vector2dU::getX)
                .addMember("Y", velocityType_inps, Vector2dU::getY)
                .register();
        vector2dUType_cartesian_ftps = this.<Vector2dU<Velocity>>buildGroupType("Vector2dU")
                .addMember("X", velocityType_ftps, Vector2dU::getX)
                .addMember("Y", velocityType_ftps, Vector2dU::getY)
                .register();
        vector2dUType_polar_ftps = this.<Vector2dU<Velocity>>buildGroupType("Vector2dU")
                .addMember("Mag", velocityType_ftps, Vector2dU::getMagnitude)
                .addMember("Angle", angleType_deg, Vector2dU::getAngle)
                .register();
        vector2dUType_cartesian_inps2 = this.<Vector2dU<Acceleration>>buildGroupType("Vector2dU")
                .addMember("X", accelerationType_inps2, Vector2dU::getX)
                .addMember("Y", accelerationType_inps2, Vector2dU::getY)
                .register();
        pose2dUType_in = this.<Pose2dU<Length>>buildGroupType("Pose2dU")
                .addMember("X", lengthType_in, Pose2dU::getX)
                .addMember("Y", lengthType_in, Pose2dU::getY)
                .addMember("Heading", angleType_deg, Pose2dU::getRotation)
                .register();
        timestampedPose2dUType_in = this.<Timestamped<Pose2dU<Length>>>buildGroupType(
                        "Timestamped Pose2dU")
                .addMember("Pose2dU", pose2dUType_in, Timestamped::getData)
                .addMember("Timestamp", timeType_s, Timestamped::getTimestamp)
                .register();

        // Other
        canBusStatusType = this.<CANBusStatus>buildGroupType("CTRE CAN Status")
                .addMember("Bus Off Count", integerType, s -> s.BusOffCount)
                .addMember("Percent Bus Utilization", floatType, s -> s.BusUtilization)
                .addMember("Receive Error Count", integerType, s -> s.REC)
                .addMember("Transmit Error Count", integerType, s -> s.TEC)
                .addMember("TX Full Count", integerType, s -> s.TxFullCount)
                .register();

        // Static Classes
        makeSyncLogEntry(
                "Robot Controller Status",
                buildGroupType("Robot Controller Status")
                        .addMember("FPGA Time (us)", longType, s -> RobotController.getFPGATime())
                        .addMember("User Button", booleanType, s -> RobotController.getUserButton())
                        .addMember("Input Voltage (V)", doubleType, s -> RobotController.getInputVoltage())
                        .addMember("Input Current (A)", doubleType, s -> RobotController.getInputCurrent())
                        .addMember("3V3 Voltage (V)", doubleType, s -> RobotController.getVoltage3V3())
                        .addMember("3V3 Current (A)", doubleType, s -> RobotController.getCurrent3V3())
                        .addMember("3V3 Fault Count", integerType, s -> RobotController.getFaultCount3V3())
                        .addMember("5V Voltage (V)", doubleType, s -> RobotController.getVoltage5V())
                        .addMember("5V Current (A)", doubleType, s -> RobotController.getCurrent5V())
                        .addMember("5V Fault Count", integerType, s -> RobotController.getFaultCount5V())
                        .addMember("6V Voltage (V)", doubleType, s -> RobotController.getVoltage6V())
                        .addMember("6V Current (A)", doubleType, s -> RobotController.getCurrent6V())
                        .addMember("6V Fault Count", integerType, s -> RobotController.getFaultCount6V())
                        .register(),
                new Object());
    }
}
