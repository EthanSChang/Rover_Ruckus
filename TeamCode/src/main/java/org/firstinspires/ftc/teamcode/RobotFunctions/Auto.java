package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.RobotFunctions.vision.MasterVision;
import org.firstinspires.ftc.teamcode.RobotFunctions.vision.SampleRandomizedPositions;

public class Auto {
    TankHardware robot;
    HardwareMap map;
    LinearOpMode opMode;
    field_position position;

    TrajectoryRunner runner;
    Pose2d stPose = new Pose2d(0, 0, 0);
    String trajectory = "";

    MasterVision vision;
    SampleRandomizedPositions goldPosition;
    int posID = 0; //if gold position is unknown, default to left position

    public enum field_position {
        blue_crater, blue_depot, red_crater, red_depot
    }

    public Auto(TankHardware robot, HardwareMap map, LinearOpMode opMode, field_position position){
        this.robot = robot;
        this.map = map;
        this.opMode = opMode;
        this.position = position;
    }

    public void init(){
        runner = new TrajectoryRunner(robot.driveTrain, opMode);

        //sets starting position
        switch(position){
            case blue_crater:
                stPose = new Pose2d(12, 12, Math.toRadians(135));
                break;
            case blue_depot:
                stPose = new Pose2d(-12, 12, Math.toRadians(-135));
                break;
            case red_crater:
                stPose = new Pose2d(-12, -12, Math.toRadians(-45));
                break;
            case red_depot:
                stPose = new Pose2d(12, -12, Math.toRadians(45));
                break;
        }

        runner.setStartingPose(stPose);

        //vision initialization code
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AT7i4ID/////AAAAGcV/BI4020ycqKtCV427Y5JV93aTLjX3SXvzzWzrXFOFSRkpFhrmm7Y/N/kNo/rZ2ZqM3MZk17j" +
                "GshCaR2EzLewC5ZoDjiitcVEhIjvPLtHpwg3e+MJ5cqcbZI/txt49FBrJOgcgBU6tDpul5NY994nLB3TTgKDnlDXWJ63Lr+d5TnfeO2tLU859wT4MJ" +
                "CZRZE89q36hmlQFo6V6bk0BK9+/Qr8aXOS3GtaLlvUMlQIwXcYePvNEHvF7q8g8D6a31VUzEdEVfQiFDV/gTtvreAbD5A2pDeGL187rMZdxkXbadG7" +
                "iP7vQKrrQmY+kaIZF9sqFAHFfgH+v+ZDYkw4YKmfEeqnIToFpvCxSOMQ3vlC0";

        vision = new MasterVision(parameters, map, true, MasterVision.TFLiteAlgorithm.INFER_LEFT);
        vision.init();
        vision.enable();

        opMode.telemetry.addData("status", "initialized");
        opMode.telemetry.update();
    }

    public void run(){/*
        opMode.resetStartTime();

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        //disables vision to save processing power
        vision.disable();
        vision.shutdown();

        switch(goldPosition){
            case LEFT: posID = 0; break;
            case CENTER: posID = 1; break;
            case RIGHT: posID = 2; break;
        }

        opMode.telemetry.addData("posID", posID);
        opMode.telemetry.update();
        */
        robot.climber.climb.setPower(0.75);
        while(opMode.opModeIsActive() && robot.climber.limHigh.getState()){}
        robot.climber.climb.setPower(0);
        /*
        //finds correct trajectory to run
        switch(position){
            case blue_crater:
                if(posID == 0){trajectory = "BlueCraterLeft";}
                else if(posID == 1){trajectory = "BlueCraterCenter";}
                else if(posID == 2){trajectory = "BlueCraterRight";}
                break;

            case blue_depot:
                if(posID == 0){trajectory = "BlueDepotLeft";}
                else if(posID == 1){trajectory = "BlueDepotCenter";}
                else if(posID == 2){trajectory = "BlueDepotRight";}
                break;

            case red_crater:
                if(posID == 0){trajectory = "RedCraterLeft";}
                else if(posID == 1){trajectory = "RedCraterCenter";}
                else if(posID == 2){trajectory = "RedCraterRight";}
                break;

            case red_depot:
                if(posID == 0){trajectory = "RedDepotLeft";}
                else if(posID == 1){trajectory = "RedDepotCenter";}
                else if(posID == 2){trajectory = "RedDepotRight";}
                break;

        }

        opMode.telemetry.addData("trajectory", trajectory);
        opMode.telemetry.update();

        //runs trajectory
        runner.runTrajectory(trajectory);

        //lower climber
        robot.climber.lower();
        */
    }
}
