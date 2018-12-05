package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous
public class TrajectorySample extends LinearOpMode {
    int[] pos = new int[4];
    //MasterVision vision;
    //SamplePositions goldPosition;
    Pose2d startingPose;
    public static int startingPos; //1 blue crater, 2 blue depot, 3 red crater, 4 red depot
    String trajectory;
    public void runOpMode() throws InterruptedException {/*
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "AT7i4ID/////AAAAGcV/BI4020ycqKtCV427Y5JV93aTLjX3SXvzzWzrXFOFSRkpFhrmm7Y/N/kNo/rZ2ZqM3MZk17jGshCaR2EzLewC5ZoDjiitcVEhIjvPLtHpwg3e+MJ5cqcbZI/txt49FBrJOgcgBU6tDpul5NY994nLB3TTgKDnlDXWJ63Lr+d5TnfeO2tLU859wT4MJCZRZE89q36hmlQFo6V6bk0BK9+/Qr8aXOS3GtaLlvUMlQIwXcYePvNEHvF7q8g8D6a31VUzEdEVfQiFDV/gTtvreAbD5A2pDeGL187rMZdxkXbadG7iP7vQKrrQmY+kaIZF9sqFAHFfgH+v+ZDYkw4YKmfEeqnIToFpvCxSOMQ3vlC0";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.init();// enables the camera overlay. this will take a couple of seconds


        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        TrajectoryRunner runner = new TrajectoryRunner(drive, this);

        if(startingPos == 1){startingPose = new Pose2d(12, 12, Math.toRadians(135));}
        else if(startingPos == 2){startingPose = new Pose2d(-12, 12, Math.toRadians(-135));}
        else if(startingPos == 3){startingPose = new Pose2d(-12, -12, Math.toRadians(-45));}
        else if(startingPos == 4){startingPose = new Pose2d(12, -12, Math.toRadians(45));}
        runner.setStartingPose(startingPose);



        waitForStart();
        vision.enable();// enables the tracking algorithms. this might also take a little time


        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive() && getRuntime() < 5){
            telemetry.addData("status:", "running");
            switch(goldPosition){
                case UNKNOWN: pos[0]++; break;
                case LEFT: pos[1]++; break;
                case CENTER: pos[2]++; break;
                case RIGHT: pos[3]++; break;
            }
            telemetry.addData("unknown", pos[0]);
            telemetry.addData("left", pos[1]);
            telemetry.addData("center", pos[2]);
            telemetry.addData("right", pos[3]);
            telemetry.update();
        }
        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.


        double max = 0;
        int maxID = 0;

        for(int i = 0; i < 4; i++){
            if(pos[i] > max){
                max = pos[i];
                maxID = i;
            }
            if(!opModeIsActive()){break;}
        }

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

        runner.runTrajectory(trajectory);*/
    }
}