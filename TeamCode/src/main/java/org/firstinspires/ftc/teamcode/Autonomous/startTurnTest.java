package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class startTurnTest extends LinearOpMode {
    DcMotor intake;

    ElapsedTime timer = new ElapsedTime();
    public void hold(double waitTime) {
        timer.reset();
        while(timer.time() < waitTime && opModeIsActive()) {

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "BIM22");


//        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40.0, 62.0, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);


        Trajectory trajectory1 = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-55.0, 60.0), Math.toRadians(180.0))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(5.0)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-60.0))))
                .forward(25.0)
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .back(8)
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end().plus(new Pose2d(0.0, 0.0, Math.toRadians(-135.0))), true)
                .splineTo(new Vector2d(-10.0, 41.0), Math.toRadians(0.0))
                .build();

        waitForStart();

        drive.followTrajectory(trajectory1);

        intake.setPower(1.0);
        hold(3.0); //spin ducks
        intake.setPower(0.0);

        drive.followTrajectory(trajectory2);

        drive.turn(Math.toRadians(-60.0));

        drive.followTrajectory(trajectory3);

        intake.setPower(1.0);
        hold(5.0); //deposit preload
        intake.setPower(0.0);

        drive.followTrajectory(trajectory4);

        drive.turn(Math.toRadians(-135.0));

        drive.followTrajectory(trajectory5);

        intake.setPower(1.0);
        hold(1.0); //raise odometry wheels
        intake.setPower(0.0);

    }
}
