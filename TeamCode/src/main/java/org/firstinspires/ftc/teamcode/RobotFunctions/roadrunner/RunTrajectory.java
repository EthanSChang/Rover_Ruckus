package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKd;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKi;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.HeadingKp;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKd;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKi;
import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.RoadRunnerConstants.PathKp;

@Autonomous
public class RunTrajectory extends LinearOpMode {//TODO: add trajectory runner class

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Trajectory trajectory;

        try {
            trajectory = AssetsTrajectoryLoader.load("BlueCraterCenter");
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

        Pose2d pose = new Pose2d(12, 12, 270 * (Math.PI / 180)); //sets starting position
        drive.setPoseEstimate(pose);
        waitForStart();

        follower.followTrajectory(trajectory);
        while (opModeIsActive()){// && follower.isFollowing()) {
            Pose2d currentPose = drive.getPoseEstimate();
            follower.update(currentPose);
            drive.updatePoseEstimate();
            packet.put("tgt", 90);
            packet.put("actual heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
