package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff.PID;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Sensors;

/**
 * This class interfaces subsystems (currently drivetrain and sensors) in a class for a robot using tank drive
 * The init function needs to be passed a linear opmode, because the opmode may be checked if it is stopped, then any active loops are stopped
 * To use: in linOpMode make a new tank hardware instance (TankHardware robot = new TankHardware();), then initiate it in the init period of the linOpMode.
 * To access a specific subsystem: write the name of the hardware instance, then add .subsystem name after it
 * Note: if you need to use functions that check the opmode, you need to be using a linear opmode add while(opModeIsActive()) to opmode to make it an iterative opmode
 *
 * @author ethan
 * TODO: clean up this class and add commonly used variables to replace hard coded numbers
 */


public class TankHardware {
    public DriveTrain driveTrain;
    public Sensors sensors;
    LinearOpMode LinOpmode;
    OpMode opmode;
    HardwareMap hMap;
    Calculators calc = new Calculators();
    double distBetweenWheels = 16.25; //inches

    public void init(HardwareMap map, LinearOpMode LinOpMode){
        this.LinOpmode = LinOpMode;
        hMap = map; //stores hardware map
        driveTrain = new DriveTrain(hMap, LinOpmode);
        driveTrain.bl.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }

    public void init(HardwareMap map){
        hMap = map; //stores hardware map
        driveTrain = new DriveTrain(hMap);
        driveTrain.bl.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        sensors = new Sensors(hMap);
    }

    PID turning = new PID(0, 0, 0, 0.5, -0.7, 0.7); //TODO: tune pid
    double startAngle, output, error, target;

    public void pidTurn(double angle){ //set to negative angle to go in opposite direction
        startAngle = sensors.getHeading();
        target = ((angle + startAngle) + 360) % 360; //prevents angle from exceeding 360

        driveTrain.setMode(DriveTrain.motor_mode.run_with_encoder); //makes motor response linear so pid can handle it better

        error = target - startAngle;
        while(LinOpmode.opModeIsActive() && (error > 1 || Math.abs(output) > 0.2)){
            error = target - sensors.getHeading();
            output = turning.getOutput(error);

            driveTrain.bl.setPower(-output);
            driveTrain.br.setPower(output);
            driveTrain.fl.setPower(-output);
            driveTrain.fr.setPower(output);
        }

    }

    Pose pos = new Pose(0, 0, 0);
    Point position = new Point(0, 0);
    double dist, leftDist, rightDist;
    double cl, cr, cfl, cfr; //current values
    double dl, dr, dfl, dfr; //delta encoder distances for motors
    double pl, pr, pfl, pfr; //previous encoder readings for all motors
    double x, y, prevX, prevY;
    double heading, prevHeading;
    private Pose CalcPose(){
        cl = driveTrain.bl.getCurrentPosition(); cr = driveTrain.br.getCurrentPosition(); cfl = driveTrain.fl.getCurrentPosition(); cfr = driveTrain.fr.getCurrentPosition();
        dl = cl - pl; dr = cr - pr; dfl = cfl - pfl; dfr = cfr - pfr;

        dist = calc.Encoder2Ft((dl + dr + dfl + dfr) / 4);
        leftDist = calc.Encoder2Ft((dl + dfl) / 2);
        rightDist = calc.Encoder2Ft((dr + dfr) / 2);

        heading = prevHeading + ((rightDist - leftDist) / (distBetweenWheels / 12));
        x = prevX - dist * Math.sin(prevHeading);
        y = prevY + dist * Math.cos(prevHeading);

        pl = cl; pr = cr; pfl = cfl; pfr = cfr;

        prevX = x; prevY = y; prevHeading = heading;

        pos.setPose(x, y, heading);

        return pos;
    }

    public Pose GetPose(){
        return CalcPose();
    }

    public Point GetPosition(){
        position.setPosition(CalcPose().getX(), CalcPose().getY());
        return position;
    }

}