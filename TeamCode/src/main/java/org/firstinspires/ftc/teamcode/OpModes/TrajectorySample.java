package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.CameraFlash;
import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot.Detectors.Sampling;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.SampleTankDrive;
@Config
@Autonomous
public class TrajectorySample extends LinearOpMode {
    Sampling detector; //cannot write as Sampling detector = new Sampling();, will cause robot controller to crash in init
    int[] pos = new int[4];
    CameraFlash flash = new CameraFlash();
    Pose2d startingPose;
    public static int startingPos; //1 blue crater, 2 blue depot, 3 red crater, 4 red depot
    String trajectory;
    public void runOpMode() throws InterruptedException {
        //flash.on();
        detector = new Sampling(); //need to add this piece during init
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, this);
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        TrajectoryRunner runner = new TrajectoryRunner(drive, this);

        if(startingPos == 1){startingPose = new Pose2d(12, 12, Math.toRadians(135));}
        else if(startingPos == 2){startingPose = new Pose2d(-12, 12, Math.toRadians(-135));}
        else if(startingPos == 3){startingPose = new Pose2d(-12, -12, Math.toRadians(-45));}
        else if(startingPos == 4){startingPose = new Pose2d(12, -12, Math.toRadians(45));}
        runner.setStartingPose(startingPose);



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

        //flash.off();

        double max = 0;
        int maxID = 0;

        for(int i = 0; i < 4; i++){
            if(pos[i] > max){
                max = pos[i];
                maxID = i;
            }
            if(!opModeIsActive()){break;}
        }
        detector.disable();
        switch(startingPos){
            case 1:
                if(maxID == 1){trajectory = "BlueCraterLeft";}
                else if(maxID == 2){trajectory = "BlueCraterCenter";}
                else if(maxID == 3){trajectory = "BlueCraterRight";}
                break;
            case 2:
                if(maxID == 1){trajectory = "BlueDepotLeft";}
                else if(maxID == 2){trajectory = "BlueDepotCenter";}
                else if(maxID == 3){trajectory = "BlueDepotRight";}
                break;
            case 3:
                if(maxID == 1){trajectory = "RedCraterLeft";}
                else if(maxID == 2){trajectory = "RedCraterCenter";}
                else if(maxID == 3){trajectory = "RedCraterRight";}
                break;
            case 4:
                if(maxID == 1){trajectory = "RedDepotLeft";}
                else if(maxID == 2){trajectory = "RedDepotCenter";}
                else if(maxID == 3){trajectory = "RedDepotRight";}
                break;
        }
        telemetry.addData("max id", maxID); //0 unknown, 1 left, 2 center, 3 right
        telemetry.addData("trajectory", trajectory);
        telemetry.update();

        runner.runTrajectory(trajectory);
    }
}