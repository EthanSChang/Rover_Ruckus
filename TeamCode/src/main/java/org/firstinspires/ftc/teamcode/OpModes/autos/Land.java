package org.firstinspires.ftc.teamcode.OpModes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.AutoTransitioner;
import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.TrajectoryRunner;


@Autonomous(name = "land", group = "auto")
public class Land extends LinearOpMode { //just lands robot and moves climber out of hook, then lowers climber
    TankHardware robot = new TankHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        AutoTransitioner.transitionOnStop(this, "ArcadeDrive"); //automatic transition to teleop program
        robot.init(hardwareMap);
        TrajectoryRunner runner = new TrajectoryRunner(robot.driveTrain, this);

        waitForStart();

        robot.climber.raise();
        runner.runTrajectory("turn90");
        robot.climber.lower();

    }
}
