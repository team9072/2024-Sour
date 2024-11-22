package frc.robot.subsystems.intake;

import org.growingstems.measurements.Measurements.Voltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.deploy.IntakeDeployHalI;
import frc.robot.subsystems.intake.rollers.IntakeRollersHalI;

public class Intake extends SubsystemBase {
    public enum IntakePosition {
        Stow(54),
        Clear(45),
        Deploy(-39);

        public final double degrees;

        IntakePosition(double positionDegrees) {
            degrees = positionDegrees;
        };
    }

    public enum IntakeSpeed {
        Intake(4),
        Eject(-4);

        public final Voltage volts;

        IntakeSpeed(double volts) {
            this.volts = new Voltage(volts);
        };
    }

    private final IntakeRollersHalI m_rollers;
    private final IntakeDeployHalI m_deploy;

    public Intake(IntakeRollersHalI rollers, IntakeDeployHalI deploy) {
        m_rollers = rollers;
        m_deploy = deploy;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot position", m_deploy.getPosition());
    }

    private void stopRollers() {
        m_rollers.brake();
    }

    private void startIntake() {
        m_rollers.setPower(IntakeSpeed.Intake.volts);
    }

    private void startEject() {
        m_rollers.setPower(IntakeSpeed.Eject.volts);
    }

    private void setPosition(IntakePosition position) {
        m_deploy.setPosition(position.degrees);
    }

    private void setDeploy() {
        setPosition(IntakePosition.Deploy);
    }

    private void setStow() {
        setPosition(IntakePosition.Deploy);
        stopRollers();
    }

    private void setClear() {
        setPosition(IntakePosition.Deploy);
        stopRollers();
    }

    public Command intake() {
        return runEnd(() -> {
            startIntake();
            setDeploy();
        }, this::setStow);
    }

    public Command whileCleared(Command command) {
        return Commands.sequence(
            runOnce(this::setClear),
            Commands.waitUntil(() -> this.m_deploy.getPosition() < IntakePosition.Clear.degrees + 1),
            command
        ).finallyDo(this::setStow);
    }
}
