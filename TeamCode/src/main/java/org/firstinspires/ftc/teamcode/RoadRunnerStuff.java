package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.ParametricCurve;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.QuinticSplineSegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

public class RoadRunnerStuff extends LinearOpMode {




    public void runOpMode(){


        Path spline = new Path(Arrays.asList(new QuinticSplineSegment(new QuinticSplineSegment.Waypoint(10, 10, 10, 0),
                new QuinticSplineSegment.Waypoint(20, 20, 0, 10)))); //first derivative is heading

    }
}
