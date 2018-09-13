package org.firstinspires.ftc.teamcode.RobotFunctions;

/**
 * This class contains methods for a pose (same as a point with the addition of heading)
 *
 * @author ethan
 */

public class Pose {
    double x, y, heading;

    public Pose(double inX, double inY, double Heading) {
        x = inX;
        y = inY;
        heading = Heading;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getHeading(){
        return heading;
    }

    public void setPose(double X, double Y, double Heading){
        x = X;
        y = Y;
        heading = Heading;
    }

    public void setX(double X){
        x = X;
    }

    public void setY(double Y){
        y = Y;
    }

    public void setHeading(double Heading){
        heading = Heading;
    }
}