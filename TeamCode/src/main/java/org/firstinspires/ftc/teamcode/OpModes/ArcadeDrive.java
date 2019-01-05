package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

import static org.firstinspires.ftc.teamcode.RobotFunctions.dashboardConstants.Misc.turnSpeed;

@TeleOp
public class ArcadeDrive extends OpMode {
    TankHardware robot = new TankHardware();

    boolean started = false; //has the robot started climbing
    boolean rtPrevSt; //last state of right bumper (for lifiting)
    double rtStick;

    public void init(){
        robot.init(hardwareMap);
        robot.driveTrain.setBrake();
    }

    public void loop() {
        //drivetrain control
        robot.driveTrain.arcadeDrive(gamepad1.left_stick_x * turnSpeed, gamepad1.left_stick_y, gamepad1.a);

        //climber control

        //lift robot automatically
        if(gamepad1.right_bumper && !rtPrevSt && !started && robot.climber.limLow.getState()) { //if bumper is pressed for the first time and the climber isn't already lifted
            started = true;
            robot.climber.climb.setPower(-0.75); //start climbing
        } else if(gamepad1.right_bumper && !rtPrevSt && started && robot.climber.limLow.getState()) { //if button is pressed again after climbing is started
            started = false; //stop climbing if button is pressed again
            robot.climber.climb.setPower(0);
        } else if(started && !robot.climber.limLow.getState()){ //is the robot done climbing
            started = false; //stop climbing
            robot.climber.climb.setPower(0);
        }
        rtPrevSt = gamepad1.right_bumper;

        telemetry.addData("started", started);
        telemetry.addData("low lim switch", robot.climber.limLow.getState());
        telemetry.addData("right bumper", gamepad1.right_bumper);
        telemetry.update();
        //manual climber control
        rtStick = -gamepad1.right_stick_y;

        if(!robot.climber.limLow.getState() && rtStick < 0){ //if trying to lower and climber is already at low limit
            rtStick = 0;
        } else if(!robot.climber.limHigh.getState() && rtStick > 0){ //if trying to raise and climber is already at high limit
            rtStick = 0;
        }

        if(!started){ //if not automatically climbing, set power
            robot.climber.climb.setPower(rtStick);
        }





    }
}
