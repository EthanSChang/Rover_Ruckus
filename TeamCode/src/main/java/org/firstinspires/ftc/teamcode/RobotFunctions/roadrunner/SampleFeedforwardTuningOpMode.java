package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SampleFeedforwardTuningOpMode extends FeedforwardTuningOpMode {
    public SampleFeedforwardTuningOpMode() {
        // TODO: change the following to match your drive
        super(72.0, SampleTankDrive.MOTOR_CONFIG.getMaxRPM(), 4.0);
    }

    @Override
    protected Drive initDrive() {
        return new SampleTankDrive(hardwareMap);
    }
}