package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

import static org.firstinspires.ftc.teamcode.RobotFunctions.DashboardConstants.HeadingKd;
import static org.firstinspires.ftc.teamcode.RobotFunctions.DashboardConstants.HeadingKi;
import static org.firstinspires.ftc.teamcode.RobotFunctions.DashboardConstants.HeadingKp;
import static org.firstinspires.ftc.teamcode.RobotFunctions.DashboardConstants.PathKd;
import static org.firstinspires.ftc.teamcode.RobotFunctions.DashboardConstants.PathKi;
import static org.firstinspires.ftc.teamcode.RobotFunctions.DashboardConstants.PathKp;

@Autonomous
public class RunTrajectory extends LinearOpMode {
    SampleTankDrive drive = new SampleTankDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory trajectory;
        try {
            trajectory = AssetsTrajectoryLoader.load("splineTest");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // TODO: tune kV, kA, and kStatic in the following follower
        // then tune the PID coefficients after you verify the open loop response is roughly correct
        TankPIDVAFollower follower = new TankPIDVAFollower(
                drive,
                new PIDCoefficients(PathKp, PathKi, PathKd),
                new PIDCoefficients(HeadingKp, HeadingKi, HeadingKd),
                0.017,
                0,
                0);

        waitForStart();

        follower.followTrajectory(trajectory);
        while (opModeIsActive() && follower.isFollowing()) {
            Pose2d currentPose = drive.getPoseEstimate();
            follower.update(currentPose);
            drive.updatePoseEstimate();
        }
    }
}
