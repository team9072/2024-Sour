package frc.robot.subsystems.intake.deploy;

public interface IntakeDeployHalI {
    public void setPosition(double positionDegrees);

    //Set the current position as the home point
    public void setHomePoint();

    public double getPosition();
    public double getVelocity();
}
