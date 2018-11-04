package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.TrajectoryRunner;
@Config
@Autonomous
public class RunTrajectory extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    public static int trajectoryNum;
    String trajectory;
    Pose2d startingPose;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        TrajectoryRunner runner = new TrajectoryRunner(drive, this);

        switch(trajectoryNum){//takes input from dashboard and turns it into a string for the trajectory runner
            case 1: trajectory = "BlueCraterLeft"; break;
            case 2: trajectory = "BlueCraterCenter"; break;
            case 3: trajectory = "BlueCraterRight"; break;
            case 4: trajectory = "BlueDepotLeft"; break;
            case 5: trajectory = "BlueDepotCenter"; break;
            case 6: trajectory = "BlueDepotRight"; break;
            case 7: trajectory = "RedCraterLeft"; break;
            case 8: trajectory = "RedCraterCenter"; break;
            case 9: trajectory = "RedCraterRight"; break;
            case 10: trajectory = "RedDepotLeft"; break;
            case 11: trajectory = "RedDepotCenter"; break;
            case 12: trajectory = "RedDepotRight"; break;
        }

        //sets starting position of robot based upon what trajectory is being run
        //TODO: figure out why trajectories 4-9 run backwards
        if(trajectoryNum == 1 || trajectoryNum == 2 || trajectoryNum == 3){startingPose = new Pose2d(12, 12, Math.toRadians(-45));}
        else if(trajectoryNum == 4 || trajectoryNum == 5 || trajectoryNum == 6){startingPose = new Pose2d(-12, 12, Math.toRadians(-135));}
        else if(trajectoryNum == 7 || trajectoryNum == 8 || trajectoryNum == 9){startingPose = new Pose2d(-12, -12, Math.toRadians(-45));}
        else if(trajectoryNum == 10 || trajectoryNum == 11 || trajectoryNum == 12){startingPose = new Pose2d(12, -12, Math.toRadians(-135));}

        runner.setStartingPose(startingPose);

        telemetry.addData("trajectory number", trajectoryNum);
        telemetry.addData("trajectory", trajectory);
        telemetry.update();

        waitForStart();

        runner.runTrajectory(trajectory);

    }
}
