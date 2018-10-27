package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.Detectors.Sampling;

@Autonomous
public class SamplingTest extends LinearOpMode {
    Sampling detector; //cannot write as Sampling detector = new Sampling();, will cause robot controller to crash in init
    public int left, center, right, unknown;

    public void runOpMode() throws InterruptedException { //currently crashes on init
        detector = new Sampling(); //need to add this piece during init
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, this);

        waitForStart();
        detector.enable();


        if(left>center && left>right && left>unknown){
            telemetry.addData("measured position", "left");
        } else if(center>left && center>right && left>unknown){
            telemetry.addData("measured position", "center");
        } else if(right>left && right>center && right>unknown){
            telemetry.addData("measured position", "right");
        } else{
            telemetry.addData("measured position", "unknown");
        }
        telemetry.update();

        while(opModeIsActive()){}

        detector.disable();
    }
}