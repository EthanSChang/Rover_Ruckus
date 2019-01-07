package org.firstinspires.ftc.teamcode.OpModes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.Auto;
import org.firstinspires.ftc.teamcode.RobotFunctions.AutoTransitioner;
import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;


@Autonomous(name = "redCrater", group = "auto")
public class redCrater extends LinearOpMode {
    TankHardware robot = new TankHardware();
    Auto auto;
    @Override
    public void runOpMode() throws InterruptedException {
        AutoTransitioner.transitionOnStop(this, "ArcadeDrive"); //automatic transition to teleop program
        //this.msStuckDetectStop = 1500; //adds more delay before robot controller crashes, default 1000 ms
        robot.init(hardwareMap, this);
        auto = new Auto(robot, hardwareMap, this, Auto.field_position.red_crater);
        auto.init();

        waitForStart();

        auto.run();
    }
}