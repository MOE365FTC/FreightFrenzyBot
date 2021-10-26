package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;

@Autonomous
public class BlueDownAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

//        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40.0, 62.0, Math.toRadians(-90.0));
        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose.plus(new Pose2d(0.0, 0.0, Math.toRadians(110.0))))
            .back(15.0)
            .build();


        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
            .splineTo(new Vector2d(-45.0, 27.0), Math.toRadians(0.0))
            .forward(2)
            .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
            .back(5)
            .splineTo(new Vector2d(-30.0, 42.0), Math.toRadians(0.0))
            .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end(), true)
            .splineTo(new Vector2d(13.0, 41.0), Math.toRadians(0.0))
            .build();
            
        waitForStart();

        drive.turn(Math.toRadians(110.0));

        drive.followTrajectory(trajectory1);

        drive.followTrajectory(trajectory2);

        //DEPOSIT PRELOAD

        drive.followTrajectory(trajectory3);

        drive.followTrajectory(trajectory4);

    }



}
