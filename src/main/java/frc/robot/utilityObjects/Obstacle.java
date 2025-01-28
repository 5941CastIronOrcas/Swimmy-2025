package frc.robot.utilityObjects;

public class Obstacle {
    public double x;
    public double y;
    public boolean isStatic;
    public double avoidanceMult;
    public Obstacle(double nx, double ny, boolean nIsStatic, double nAvoidanceMult) {
        x = nx;
        y = ny;
        isStatic = nIsStatic;
        avoidanceMult = nAvoidanceMult;
    }
}
