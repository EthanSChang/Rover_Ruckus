package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    HardwareMap map;
    LinearOpMode linOpMode;
    public DigitalChannel limLow, limHigh; //false when pressed

    public DcMotorEx climb; //positive power moves climber counterclockwise
    public Climber(HardwareMap hmap, LinearOpMode linOpMode){
        map = hmap;
        this.linOpMode = linOpMode;
        climb = (DcMotorEx) map.dcMotor.get("climb");
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limLow = map.digitalChannel.get("limLow");
        limHigh = map.digitalChannel.get("limHigh");
    }

    public Climber(HardwareMap hmap){
        map = hmap;
        climb = (DcMotorEx) map.dcMotor.get("climb");
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limLow = map.digitalChannel.get("limLow");
        limHigh = map.digitalChannel.get("limHigh");
    }

    public void raise(){
        climb.setPower(-0.75);
        while(linOpMode.opModeIsActive() && limHigh.getState()){}
        climb.setPower(0);
    }

    public void lower(){
        climb.setPower(0.75);
        while(linOpMode.opModeIsActive() && limLow.getState()){}
        climb.setPower(0);
    }
}
