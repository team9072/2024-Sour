package frc.robot.subsystems.shooter.wheels;

public class ShooterWheelsDummyHal implements ShooterWheelsHalI {
    private double m_fakeSpeed = 0;

    @Override
    public void brake() {
        // NOP
    }

    @Override
    public void coast() {
        // NOP
    }

    @Override
    public void setSpeed(double speedRpm) {
        m_fakeSpeed = speedRpm;
    }

    @Override
    public void setVoltageSysId(double volts, boolean top) {
        // NOP
    }

    @Override
    public double getTopSpeed() {
        return m_fakeSpeed;
    }

    @Override
    public double getBottomSpeed() {
        return m_fakeSpeed;
    }
}
