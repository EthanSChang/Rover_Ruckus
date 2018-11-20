package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.Detectors.Sampling;

@Autonomous
public class SamplingTest extends LinearOpMode {
    Sampling detector; //cannot write as Sampling detector = new Sampling();, will cause robot controller to crash in init
    int[] pos = new int[4];
    public void runOpMode() throws InterruptedException {
        CameraDevice.getInstance().setFlashTorchMode(true); //turns on camera flash
        detector = new Sampling(); //need to add this piece during init
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, this);

        waitForStart();
        detector.enable();
        while(opModeIsActive() && getRuntime() < 5){
            telemetry.addData("status:", "running");
            switch(detector.pos){
                case 0: pos[0]++; break;
                case 1: pos[1]++; break;
                case 2: pos[2]++; break;
                case 3: pos[3]++; break;
            }
            telemetry.addData("unknown", pos[0]);
            telemetry.addData("left", pos[1]);
            telemetry.addData("center", pos[2]);
            telemetry.addData("right", pos[3]);
            telemetry.update();
        }

        CameraDevice.getInstance().setFlashTorchMode(false);

        double max = 0;
        int maxID = 0;

        for(int i = 0; i < 4; i++){
            if(pos[i] > max){
                max = pos[i];
                maxID = i;
            }
            if(!opModeIsActive()){break;}
        }
        telemetry.addData("max id", maxID);

        switch(maxID){
            case 0: telemetry.addData("detected position", "unknown");
            case 1: telemetry.addData("detected position", "left");
            case 2: telemetry.addData("detected position", "center");
            case 3: telemetry.addData("detected position", "right");
        }

        telemetry.update();
        while(opModeIsActive()){

        }

        detector.disable();
    }
}