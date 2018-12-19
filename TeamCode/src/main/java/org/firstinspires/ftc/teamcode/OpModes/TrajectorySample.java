package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.SampleTankDrive;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.RobotFunctions.vision.MasterVision;
import org.firstinspires.ftc.teamcode.RobotFunctions.vision.SampleRandomizedPositions;


@Config
@Autonomous
public class TrajectorySample extends LinearOpMode {
    MasterVision vision;
    SampleRandomizedPositions goldPosition;
    int[] pos = new int[4];
    public static int startingPos;
    TankHardware robot = new TankHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        String trajectory = "";
        TrajectoryRunner runner = new TrajectoryRunner(robot.driveTrain, this);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "AT7i4ID/////AAAAGcV/BI4020ycqKtCV427Y5JV93aTLjX3SXvzzWzrXFOFSRkpFhrmm7Y/N/kNo/rZ2ZqM3MZk17j" +
                "GshCaR2EzLewC5ZoDjiitcVEhIjvPLtHpwg3e+MJ5cqcbZI/txt49FBrJOgcgBU6tDpul5NY994nLB3TTgKDnlDXWJ63Lr+d5TnfeO2tLU859wT4MJ" +
                "CZRZE89q36hmlQFo6V6bk0BK9+/Qr8aXOS3GtaLlvUMlQIwXcYePvNEHvF7q8g8D6a31VUzEdEVfQiFDV/gTtvreAbD5A2pDeGL187rMZdxkXbadG7" +
                "iP7vQKrrQmY+kaIZF9sqFAHFfgH+v+ZDYkw4YKmfEeqnIToFpvCxSOMQ3vlC0";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_LEFT);
        vision.init();// enables the camera overlay. this will take a couple of seconds

        waitForStart();
        resetStartTime();
        vision.enable();// enables the tracking algorithms. this might also take a little time
        telemetry.addData("status", "starting");

        while(opModeIsActive() && getRuntime() < 5){
            goldPosition = vision.getTfLite().getLastKnownSampleOrder();
            telemetry.addData("goldPosition is", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    pos[0]++;
                    break;
                case CENTER:
                    pos[1]++;
                    break;
                case RIGHT:
                    pos[2]++;
                    break;
                case UNKNOWN:
                    pos[3]++;
                    break;
            }
            telemetry.addData("left count", pos[0]);
            telemetry.addData("center count", pos[1]);
            telemetry.addData("right count", pos[2]);
            telemetry.addData("unknown count", pos[3]);
            telemetry.update();
        }
        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.
        vision.shutdown();

        double max = 0;
        int maxID = 0;

        for(int i = 0; i < 4; i++){
            if(pos[i] > max){
                max = pos[i];
                maxID = i;
            }
        }

        switch(startingPos){
            case 1:
                if(maxID == 0){trajectory = "BlueCraterLeft";}
                else if(maxID == 1){trajectory = "BlueCraterCenter";}
                else if(maxID == 2){trajectory = "BlueCraterRight";}
                break;
            case 2:
                if(maxID == 0){trajectory = "BlueDepotLeft";}
                else if(maxID == 1){trajectory = "BlueDepotCenter";}
                else if(maxID == 2){trajectory = "BlueDepotRight";}
                break;
            case 3:
                if(maxID == 0){trajectory = "RedCraterLeft";}
                else if(maxID == 1){trajectory = "RedCraterCenter";}
                else if(maxID == 2){trajectory = "RedCraterRight";}
                break;
            case 4:
                if(maxID == 0){trajectory = "RedDepotLeft";}
                else if(maxID == 1){trajectory = "RedDepotCenter";}
                else if(maxID == 2){trajectory = "RedDepotRight";}
                break;
        }
        telemetry.addData("max id", maxID); //0 unknown, 1 left, 2 center, 3 right
        telemetry.addData("trajectory", trajectory);
        telemetry.update();

        runner.runTrajectory(trajectory);
    }
}