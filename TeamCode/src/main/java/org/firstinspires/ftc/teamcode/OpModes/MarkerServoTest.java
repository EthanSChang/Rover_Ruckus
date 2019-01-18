package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

@TeleOp
@Disabled
public class MarkerServoTest extends OpMode {
    TankHardware robot = new TankHardware();

    public void init(){
        robot.init(hardwareMap);
    }
    public void loop(){
        robot.markerDropper.dropper.setPosition(gamepad1.right_stick_y);
        telemetry.addData("position", gamepad1.right_stick_y);
    }
}