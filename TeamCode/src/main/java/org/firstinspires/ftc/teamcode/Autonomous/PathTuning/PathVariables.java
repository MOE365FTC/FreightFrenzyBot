package org.firstinspires.ftc.teamcode.Autonomous.PathTuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PathVariables extends LinearOpMode {

    //POINTS

    Pose2d startPosition = new Pose2d(-40.0, 62.0, Math.toRadians(0.0));

    Vector2d traj1point = new Vector2d(-55.0, 60.0);
    double traj1heading = Math.toRadians(-180.0);

    double traj2forward = 5.0;
    double traj2turn = Math.toRadians(-60.0);

    double traj3forward = 25.0;

    double traj4back = 8.0;
    double traj4turn = Math.toRadians(-150.0);

    Vector2d traj5point = new Vector2d(-8.0, 25.0);
    double traj5heading = Math.toRadians(-15.0);

    //FUNCTIONS

    ElapsedTime timer = new ElapsedTime();
    public void hold(double waitTime) {
        timer.reset();
        while(timer.time() < waitTime && opModeIsActive()) {

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
