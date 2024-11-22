package frc.robot.subsystems.intake.deploy;

public interface IntakeDeployHalI {
    public void setPosition(double degrees);
    //Set thecurrent position as the home point
    public double setHomePoint();
    public double getVelocity();
    // Retursn postion error between actual and set position
    public double getError();
}
