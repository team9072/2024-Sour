package frc.robot.subsystems.intake.deploy;

public class IntakeDeployDummyHal implements IntakeDeployHalI {
    private double m_fakePosition = 0;

    @Override
    public void setPosition(double positionDegrees) {
        m_fakePosition = positionDegrees;
    }

    @Override
    public void setHomePoint() {
        // NOP
    }

    @Override
    public double getPosition() {
        return m_fakePosition;
    }

    @Override
    public double getVelocity() {
        return 0;
    }
    
}
