package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.AutoTransitioner;

@Autonomous
public class AutoTransitionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoTransitioner.transitionOnStop(this, "Concept: TensorFlow Object Detection");
        telemetry.addData("status", "initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("runtime", getRuntime());
            telemetry.update();
        }
    }
}
