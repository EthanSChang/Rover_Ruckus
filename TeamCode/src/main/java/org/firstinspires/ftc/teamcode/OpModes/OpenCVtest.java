package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.Detectors.TestDetector;

@Autonomous
public class OpenCVtest extends LinearOpMode {
    TestDetector opencv;

    public void runOpMode() throws InterruptedException {
        opencv = new TestDetector();
        opencv.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);

        waitForStart();
        opencv.enable();

        while (opModeIsActive()) {

        }

        opencv.disable();
    }
}