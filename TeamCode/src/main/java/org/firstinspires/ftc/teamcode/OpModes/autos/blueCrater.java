package org.firstinspires.ftc.teamcode.OpModes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RobotFunctions.Auto;
import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.SampleTankDrive;
import org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.RobotFunctions.vision.MasterVision;
import org.firstinspires.ftc.teamcode.RobotFunctions.vision.SampleRandomizedPositions;


@Config
@Autonomous(name = "blueCrater", group = "auto")
public class blueCrater extends LinearOpMode {
    TankHardware robot = new TankHardware();
    Auto auto;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        auto = new Auto(robot, hardwareMap, this, Auto.field_position.blue_crater);

        waitForStart();

        auto.run();
    }
}