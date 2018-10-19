package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;
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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        //Trajectory trajectory;
        /*
        try {
            trajectory = AssetsTrajectoryLoader.load("PointTurnTest");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }*/

        DriveConstraints baseConstraints = new DriveConstraints(20.0, 30.0, Math.PI / 2, Math.PI / 2);
        TankConstraints constraints = new TankConstraints(baseConstraints, drive.getTrackWidth());
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(0, 0, 0), constraints)
                .turnTo(Math.PI)
                .build();

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
