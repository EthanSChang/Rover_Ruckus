package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.roadrunner.drive.TankDrive;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SampleTankDrive extends TankDrive {
    // TODO: change your drive motor
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);

    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();


    /**
     * These were good velocity PID values for a ~40lb robot with 1:1 belt-driven wheels off AM
     * orbital 20s. Adjust accordingly (or tune them yourself, see
     * https://github.com/acmerobotics/relic-recovery/blob/master/TeamCode/src/main/java/com/acmerobotics/relicrecovery/opmodes/tuner/DriveVelocityPIDTuner.java
     */
    public static final PIDCoefficients NORMAL_VELOCITY_PID = new PIDCoefficients(20, 8, 12);

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    public SampleTankDrive(HardwareMap hardwareMap) {
        // TODO: test running feed forward opmode with different speeds and number of turns
        super(1); //22.5 provides accurate turns

        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, at least tune kStatic and kA potentially
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, NORMAL_VELOCITY_PID);
        }

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
    }

    private static double encoderTicksToInches(int ticks) {
        Calculators calc = new Calculators();
        // TODO: modify this appropriately
        // wheel radius * radians/rev * wheel revs/motor revs * motor revs
        return calc.Encoder2Inches(ticks);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        double leftAvg;
        double rightAvg;
        leftAvg = (encoderTicksToInches(motors.get(0).getCurrentPosition()) + encoderTicksToInches(motors.get(1).getCurrentPosition())) / 2;
        rightAvg =  (encoderTicksToInches(motors.get(2).getCurrentPosition()) + encoderTicksToInches(motors.get(3).getCurrentPosition())) / 2;
        wheelPositions.add(leftAvg);
        wheelPositions.add(rightAvg);
        return wheelPositions;
    }

    public void setMotorPowers(double leftPow, double rightPow){
        leftFront.setPower(leftPow);
        leftRear.setPower(leftPow);
        rightRear.setPower(rightPow);
        rightFront.setPower(rightPow);
    }
}