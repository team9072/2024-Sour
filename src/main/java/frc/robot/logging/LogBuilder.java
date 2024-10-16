/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.robot.logging;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Telemetry;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.util.function.Consumer;
import org.growingstems.frc.util.WpiTimeSource;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.RateCalculator;
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
import org.growingstems.util.logger.AsyncLogEntry;
import org.growingstems.util.logger.DataHiveLogger;
import org.growingstems.util.logger.LogEntryType;

public class LogBuilder extends DataHiveLogger.Builder {
    private static final int k_maxFileAttempts = 10;
    private static final int k_pipeSize = 65536;
    private static final Time k_writeSleepTime = Time.milliseconds(250.0);
    private static final int k_dumpSize = 0;

    private long m_totalTransferred = 0;
    private RateCalculator m_fileWriteRate = new RateCalculator(new WpiTimeSource());

    // Helpful initializers
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

    // If defaultValue is `null`, the first value will be logged no matter what.
    // If defaultValue is not null, nothing will be logged until a different value
    // is received
    // Note: This is only valid for Async Log Entries. We may have to change the
    // input type to Consumer<T> to allow for more behavior options in the future,
    // but for now, for safety, this is explicitly requiring an AsyncLogEntry<T>
    public static <T> Consumer<T> makeNoRepeats(AsyncLogEntry<T> asyncConsumer, T defaultValue) {
        return new Consumer<T>() {
            private T prev = defaultValue;

            @Override
            public void accept(T value) {
                // Only log when error code changes
                if (prev != null && value.equals(prev)) {
                    return;
                }

                asyncConsumer.accept(value);
                prev = value;
            }
        };
    }

    public <T> Consumer<T> makeNoRepeatsAsyncEntry(
            String name, LogEntryType<T> entryType, T defaultValue) {
        return makeNoRepeats(makeAsyncLogEntry(name, entryType), defaultValue);
    }

    protected final File m_logDir;
    protected final PipedOutputStream m_outputStream;
    protected File m_logFile;

    public LogBuilder(File logDir, String initialLogFileName) {
        this(logDir, initialLogFileName, new PipedOutputStream());
    }

    private LogBuilder(File logDir, String initialLogFileName, PipedOutputStream outputStream) {
        super(outputStream, new WpiTimeSource(), "2024 Crescendo Log File");

        m_logDir = logDir;

        m_outputStream = outputStream;
        m_logFile = deconflictFilename(initialLogFileName);

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

    @Override
    public DataHiveLogger init() throws IOException {
        // Ensure log directory exists
        m_logDir.mkdir();

        @SuppressWarnings(
                "resource") // We never close it because we continue logging until the robot turns off
        var inputStream = new PipedInputStream(m_outputStream, k_pipeSize);
        @SuppressWarnings(
                "resource") // We never close it because we continue logging until the robot turns off
        var fileStream = new FileOutputStream(m_logFile);
        Telemetry.TeleRobot.logFile.setString(m_logFile.getName());
        // Make separate thread transfer data to writing stream
        new Thread() {
            public void run() {
                while (true) {
                    try {
                        var avail = inputStream.available();
                        if (avail > k_dumpSize) {
                            fileStream.write(inputStream.readNBytes(avail));

                            m_totalTransferred += avail;
                            var rate = m_fileWriteRate.update((double) m_totalTransferred);

                            Telemetry.TeleRobot.loggedBytes.setInteger(avail);
                            Telemetry.TeleRobot.loggedTotalBytesKB.setDouble(
                                    (double) m_totalTransferred / 1000.0);
                            Telemetry.TeleRobot.loggedKBPerSecond.setDouble(rate.asKilohertz());
                        }
                    } catch (IOException e) {
                        DriverStation.reportError("Error occurred while writing", e.getStackTrace());
                    }

                    try {
                        Thread.sleep((long) k_writeSleepTime.asMilliseconds());
                    } catch (InterruptedException e) {
                        DriverStation.reportError("Log thread interrupted while sleeping", e.getStackTrace());
                    }
                }
            }
        }.start();

        return super.init();
    }

    private File stringToFileHandle(String logName) {
        return new File(m_logDir, logName + ".dhl");
    }

    /**
     * Creates a File object for a new file with the given name.. DO NOT INCLUDE THE EXTENSION
     *
     * @param logName The extension-less log file name
     * @return A File object that is not already used (unless the max attempts are used)
     */
    private File deconflictFilename(String logName) {
        int i = 0;
        File file = stringToFileHandle(logName);
        do {
            if (file.exists()) {
                file = stringToFileHandle(logName + "_" + Integer.toString(++i));
            } else {
                return file;
            }
        } while (i < k_maxFileAttempts);
        return stringToFileHandle(logName + "_MAX");
    }

    /**
     * Renames the log file. DO NOT INCLUDE THE EXTENSION
     *
     * @param logName The extension-less log file name
     * @return True if the rename was successful
     */
    public boolean renameTo(String logName) {
        File newFile = deconflictFilename(logName);
        boolean success = m_logFile.renameTo(newFile);
        if (success) {
            m_logFile = newFile;
            Telemetry.TeleRobot.logFile.setString(m_logFile.getName());
        }
        return success;
    }
}
