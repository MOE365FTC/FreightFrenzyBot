package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
@Autonomous
public class BlueUpAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(5.0, 62.0, Math.toRadians(-90.0));
        drive.setPoseEstimate(startPose);

        Trajectory trajectory0 = drive.trajectoryBuilder(startPose)
                .forward(5)
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(trajectory0.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(100.0))))
                .back(45.0)
                .build();


        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineTo(new Vector2d(-45.0, 27.0), Math.toRadians(0.0))
                .forward(10)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .back(6)
                .splineTo(new Vector2d(-30.0, 42.0), Math.toRadians(0.0))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end(), true)
                .splineTo(new Vector2d(13.0, 41.0), Math.toRadians(0.0))
                .build();

        waitForStart();

        drive.followTrajectory(trajectory0);

        drive.turn(Math.toRadians(100.0));

        drive.followTrajectory(trajectory1);

        drive.followTrajectory(trajectory2);

        //DEPOSIT PRELOAD

        drive.followTrajectory(trajectory3);

        drive.followTrajectory(trajectory4);
    }
}
