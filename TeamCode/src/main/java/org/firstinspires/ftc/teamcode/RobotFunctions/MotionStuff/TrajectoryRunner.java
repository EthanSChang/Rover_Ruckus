package org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.AssetsTrajectoryLoader;

import java.io.IOException;

import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKd;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKi;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKp;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKd;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKi;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKp;

public class TrajectoryRunner {
    Trajectory Trajectory;
    TankDrive drive;
    Pose2d startingPose;
    LinearOpMode opMode;
    public TrajectoryRunner(TankDrive drive, Pose2d startingPose, LinearOpMode opMode){
        this.drive = drive;
        this.startingPose = startingPose;
        this.opMode = opMode;
    }

    public void runTrajectory(String trajectory){
        TankPIDVAFollower follower = new TankPIDVAFollower(
                drive,
                new PIDCoefficients(PathKp, PathKi, PathKd),
                new PIDCoefficients(HeadingKp, HeadingKi, HeadingKd),
                0.017,
                0,
                0);

        try {
            Trajectory = AssetsTrajectoryLoader.load(trajectory);
        } catch (IOException e){
            throw new RuntimeException(e);
        }

        drive.setPoseEstimate(startingPose);

        follower.followTrajectory(Trajectory);

        while(follower.isFollowing() && opMode.opModeIsActive()){
            Pose2d currentPose = drive.getPoseEstimate();
            follower.update(currentPose);
            drive.updatePoseEstimate();
        }
    }


}
