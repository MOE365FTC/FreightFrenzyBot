package org.firstinspires.ftc.teamcode.Autonomous.PathTuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PathVariables extends LinearOpMode {

    //POINTS: DOWN AUTONS

    public Pose2d startPosition = new Pose2d(-40.0, 62.0, Math.toRadians(0.0));

    public double traj1x = -55.0;
    public double traj1y = 60.0;
    public double traj1heading = Math.toRadians(-180.0);

    public double traj2forward = 5.0;
    public double traj2turn = Math.toRadians(-60.0);

    public double traj3forward = 25.0;

    public double traj4back = 8.0;
    public double traj4turn = Math.toRadians(-120.0);

    public double traj5back = 38.0;

    //POINTS: UP AUTONS

    public Pose2d startPositionUp = new Pose2d(14.0, 62.0, Math.toRadians(-180.0));

    public double UpTraj1turn = Math.toRadians(45.0);
    public double UpTraj1forward = 14.0;

    public double UpTraj2back = -8.0;
    public double UpTraj2turn = Math.toRadians(-45.0);

    public double UpTraj3back = 10.0;

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
