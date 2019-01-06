package org.firstinspires.ftc.teamcode.OpModes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.Auto;
import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;


@Autonomous(name = "blueCrater", group = "auto")
public class blueCrater extends LinearOpMode {
    TankHardware robot = new TankHardware();
    Auto auto;
    @Override
    public void runOpMode() throws InterruptedException {
        //this.msStuckDetectStop = 1500; //adds more delay before robot controller crashes, default 1000 ms
        robot.init(hardwareMap, this);
        auto = new Auto(robot, hardwareMap, this, Auto.field_position.blue_crater);
        auto.init();

        waitForStart();

        auto.run();
    }
}