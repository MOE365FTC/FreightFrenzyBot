package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.PathTuning.PathVariables;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
@Autonomous
public class BlueUpAuton extends PathVariables {
    DcMotor intake;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "BIM22");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = startPositionUp;
        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose.plus(new Pose2d(0.0, 0.0, UpTraj1turn)))
                .forward(UpTraj1forward)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .back(UpTraj2back)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0.0, 0.0, UpTraj2turn)))
                .back(UpTraj3back)
                .build();

        waitForStart();

        drive.turn(UpTraj1turn);

        drive.followTrajectory(trajectory1);

        //DEPOSIT

        drive.followTrajectory(trajectory2);

        drive.turn(UpTraj2turn);

        drive.followTrajectory(trajectory3);

        intake.setPower(1.0);
        hold(1.0); //raise odometry wheels
        intake.setPower(0.0);

        drive.manualDrive(-1.0);
        hold(1.0);
        drive.manualDrive(0.0);

        drive.manualTurn(-1.0, 1.0);
        hold(0.25);
        drive.manualTurn(0.0,0.0);

        intake.setPower(1.0);
        drive.manualDrive(-1.0);
        hold(1.0);
        drive.manualDrive(0.0);
        hold(0.2);
        intake.setPower(0.0);

        drive.manualTurn(1.0, -1.0);
        hold(0.25);
        drive.manualTurn(0.0,0.0);

        drive.manualDrive(1.0);
        hold(1.0);
        drive.manualDrive(0.0);
    }
}
