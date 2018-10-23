package org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.Detectors;

import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.OpenCVpipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.OpenCV.HighH;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.OpenCV.HighS;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.OpenCV.HighV;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.OpenCV.LowH;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.OpenCV.LowS;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.OpenCV.LowV;

/**
 * This class is a opencv detector for detecting where the gold mineral is to sample
 */

public class Sampling extends OpenCVpipeline {
    private Mat rgba = new Mat();
    private Mat hsv = new Mat();
    private Mat blurred = new Mat();
    private Mat filtered = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    private double contourMin = 500;
    private double contourMax = 500000;
    private int contourId;
    private double xPos;
    private double imageWidth = 1280;
    private position pos;

    public enum position {
        left, center, right, unknown
    }
    //TODO: add feature to set limits or margins to where the mineral can be in each position (for example, mineral can be within 20 < x < 50)
    public Mat processFrame(Mat rgba, Mat gray){
        this.rgba = rgba;
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(hsv, blurred, new Size(5, 5));
        Core.inRange(blurred, new Scalar(LowH, LowS, LowV), new Scalar(HighH, HighS, HighV), filtered); //low 20, 80, 200 high 30, 255, 255
        Imgproc.findContours(filtered, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        int i = 0;
        double maxArea = 0;
        double contourArea = 0;
        Rect boundingRect = new Rect();
        while(opmode.opModeIsActive() && i < contours.size()){ //made for loop out of while loop because I need to check if opmode is still running
            MatOfPoint cnts = contours.get(i);
            contourArea = Imgproc.contourArea(cnts);

            if(contourArea > contourMin && contourArea < contourMax){
                if(contourArea > maxArea){
                    maxArea = contourArea;
                    contourId = i;
                }
            }
            i++;
        }

        if(maxArea > 0){ //checks if there is a contour to draw
            Imgproc.drawContours(rgba, contours, contourId, new Scalar(0, 255, 0), 3);
            boundingRect = Imgproc.boundingRect(contours.get(contourId));
            Imgproc.rectangle(rgba, boundingRect.tl(), boundingRect.br(), new Scalar(0, 0, 255), 3);
            opmode.telemetry.addData("contour id", Imgproc.contourArea(contours.get(contourId)));
        }

        xPos = boundingRect.tl().x + (1/2 * boundingRect.width);

        if(xPos < rgba.width() / 3 && xPos > 0){
            pos = position.left;
            addText("left");
            opmode.telemetry.addData("position", "left");
        } else if(xPos < rgba.width() / 3 * 2 && xPos > rgba.width() / 3){
            pos = position.center;
            addText("center");
            opmode.telemetry.addData("position", "center");
        } else if(xPos > rgba.width() / 3 * 2){
            pos = position.right;
            addText("right");
            opmode.telemetry.addData("position", "right");
        } else {
            pos = position.unknown;
            addText("unknown");
            opmode.telemetry.addData("position", "unknown");
        }

        opmode.telemetry.update();

        contours.clear();

        return rgba;

    }

    public void addText(String text){
        Imgproc.putText(rgba, text, new Point(0, 130), 0, 5, new Scalar(0, 255, 255), 5);
    }

    public position getPosition(){
        return pos;
    }
}
