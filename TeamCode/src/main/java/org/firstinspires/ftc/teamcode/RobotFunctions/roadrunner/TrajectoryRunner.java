package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.AssetsTrajectoryLoader;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.DashboardUtil;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.DriveConstants;

import java.io.IOException;

import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKd;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKi;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKp;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKd;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKi;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKp;

public class TrajectoryRunner {
    TankDrive drive;
    Pose2d startingPose = new Pose2d(0, 0, 0);
    LinearOpMode opMode;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    public TrajectoryRunner(TankDrive drive, Pose2d startingPose, LinearOpMode opMode){
        this.drive = drive;
        this.startingPose = startingPose;
        this.opMode = opMode;
    }

    public TrajectoryRunner(TankDrive drive, LinearOpMode opMode){
        this.drive = drive;
        this.opMode = opMode;
    }

    public void setStartingPose(Pose2d startingPose){
        this.startingPose = startingPose;
    }

    public void runTrajectory(String trajectory){
        Trajectory Trajectory;
        TankPIDVAFollower follower = new TankPIDVAFollower(
                drive,
                new PIDCoefficients(PathKp, PathKi, PathKd),
                new PIDCoefficients(HeadingKp, HeadingKi, HeadingKd),
                DriveConstants.kV,
                0,
                0);

        if(trajectory != null){ //TODO: test null checking
            try {
                Trajectory = AssetsTrajectoryLoader.load(trajectory);
            } catch (IOException e){
                throw new RuntimeException(e);
            }

            drive.setPoseEstimate(startingPose);

            follower.followTrajectory(Trajectory);
        } else {
            opMode.telemetry.addData("error:", "trajectory is null");
        }

        while(follower.isFollowing() && opMode.opModeIsActive()){
            Pose2d currentPose = drive.getPoseEstimate();

            packet.put("x pos", drive.getPoseEstimate().getX());
            packet.put("y pos", drive.getPoseEstimate().getY());
            dashboard.sendTelemetryPacket(packet);

            follower.update(currentPose);
            drive.updatePoseEstimate();
        }
    }


}
