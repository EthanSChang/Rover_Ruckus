package org.firstinspires.ftc.teamcode.RobotFunctions;

/**
 * This class contains methods for basic calculations such as converting encoder counts to distance in inches
 *
 * TODO: delete and replace some methods (encoder calculations) and put into drivetrain class
 */

public class Calculators {
    Variables variables = new Variables();
    double encoderDistance;
    double distanceIn;
    double distanceFt;

    public double Encoder2Inches(double encoder){
        encoderDistance = encoder;
        distanceIn = (encoderDistance / variables.encoderCntsPerRev * variables.gearRatio) * (variables.wheelDiameter * Math.PI);
        return distanceIn;
    }

    public double Encoder2Ft(double encoder){
        encoderDistance = encoder;
        distanceFt = (encoderDistance / variables.encoderCntsPerRev * variables.gearRatio) * (variables.wheelDiameter * Math.PI) / 12;
        return distanceFt;
    }

    int encoder;
    public int Inches2Encoder(double inches){
        encoder = (int) Math.round(inches / (variables.wheelDiameter * Math.PI) / variables.gearRatio * variables.encoderCntsPerRev);
        return encoder;
    }

    Point p1, p2;
    double distance;

    public double PointDistance(Point point1, Point point2){
        p1 = point1;
        p2 = point2;

        distance = Math.sqrt((Math.pow((p2.getX() - p1.getX()), 2)) + (Math.pow((p2.getY() - p1.getY()), 2))); // distance formula
        return distance;
    }


}
