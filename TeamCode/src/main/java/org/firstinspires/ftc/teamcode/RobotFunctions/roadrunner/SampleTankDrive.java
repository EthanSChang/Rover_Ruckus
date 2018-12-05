package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.RobotFunctions.Calculators;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@Config
public class SampleTankDrive extends TankDrive {
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class);
    public BNO055IMU imu;

    public static com.acmerobotics.roadrunner.control.PIDCoefficients TRANSLATIONAL_PID = new com.acmerobotics.roadrunner.control.PIDCoefficients(0, 0, 0);
    public static com.acmerobotics.roadrunner.control.PIDCoefficients HEADING_PID = new com.acmerobotics.roadrunner.control.PIDCoefficients(0, 0, 0);

    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();


    /**
     * These were good velocity PID values for a ~40lb robot with 1:1 belt-driven wheels off AM
     * orbital 20s. Adjust accordingly (or tune them yourself, see
     * https://github.com/acmerobotics/relic-recovery/blob/master/TeamCode/src/main/java/com/acmerobotics/relicrecovery/opmodes/tuner/DriveVelocityPIDTuner.java
     */
    //public static final com.qualcomm.robotcore.hardware.PIDCoefficients NORMAL_VELOCITY_PID = new com.qualcomm.robotcore.hardware.PIDCoefficients(15, 2, 9);
        
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors, leftMotors, rightMotors;
    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    public SampleTankDrive(HardwareMap hardwareMap) {
        // TODO: test running feed forward opmode with different speeds and number of turns
        super(DriveConstants.TRACK_WIDTH); //20.16 for home drivetrain
        constraints = new TankConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        follower = new TankPIDVAFollower(this, TRANSLATIONAL_PID, HEADING_PID,
                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);

        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, at least tune kStatic and kA potentially
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        leftMotors = Arrays.asList(leftFront, leftRear);
        rightMotors = Arrays.asList(rightFront, rightRear);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftMotors.get(0).getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    public Pose2d getFollowingError() {
        return follower.getLastError();
    }

    public void updateFollower() {
        follower.update(getPoseEstimate());
    }

    public void update() {
        updatePoseEstimate();
        updateFollower();
    }

    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    private static double encoderTicksToInches(int ticks) {
        Calculators calc = new Calculators();
        return calc.Encoder2Inches(ticks);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += DriveConstants.encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += DriveConstants.encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }


    public void setMotorPowers(double leftPow, double rightPow){
        leftFront.setPower(leftPow);
        leftRear.setPower(leftPow);
        rightRear.setPower(rightPow);
        rightFront.setPower(rightPow);
    }
}