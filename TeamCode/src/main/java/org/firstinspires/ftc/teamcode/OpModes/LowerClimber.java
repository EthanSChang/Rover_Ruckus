package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

@Autonomous
public class LowerClimber extends LinearOpMode {
    TankHardware robot = new TankHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.climber.lower();
    }
}
