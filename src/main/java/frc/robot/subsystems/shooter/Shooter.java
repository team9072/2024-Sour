package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.wheels.ShooterWheelsHalI;

public class Shooter extends SubsystemBase {
    private ShooterWheelsHalI m_wheels;

    private final SysIdRoutine m_topWheelsSysId = wheelsSysId(true);
    private final SysIdRoutine m_bottomWheelsSysId = wheelsSysId(false);

    private SysIdRoutine wheelsSysId(boolean top) {
        String stateKey = "Shooter Wheels (" + (top ? "Top" : "Bottom") + ") SysId State";
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                        null, // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString(stateKey, state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_wheels.setVoltageSysId(volts.in(Volts), top), null, this));
    }

    public Shooter(ShooterWheelsHalI wheels) {
        m_wheels = wheels;
    }

    private Command spindownWheelsSysId() {
        return Commands.sequence(
                Commands.waitSeconds(2), runOnce(m_wheels::brake), Commands.waitSeconds(1));
    }

    public Command topWheelsSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_topWheelsSysId.quasistatic(direction).andThen(spindownWheelsSysId());
    }

    public Command topWheelsSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_topWheelsSysId.dynamic(direction).andThen(spindownWheelsSysId());
    }

    public Command bottomWheelsSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_bottomWheelsSysId.quasistatic(direction).andThen(spindownWheelsSysId());
    }

    public Command bottomWheelsSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_bottomWheelsSysId.dynamic(direction).andThen(spindownWheelsSysId());
    }
}
