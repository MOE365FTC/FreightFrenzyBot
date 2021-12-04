package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.PathTuning.PathVariables;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;

@Autonomous
public class RedDownAuton extends PathVariables {
  DcMotor intake;



  @Override
  public void runOpMode() throws InterruptedException {
    intake = hardwareMap.get(DcMotor.class, "BIM22");

//        ONCE WE GET THE ACTUAL ROBOT CHECK GOOGLE DOC NAMED: UNCORRECTEDAUTON
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPose = new Pose2d(-40.0, -62.0, Math.toRadians(0.0));
    drive.setPoseEstimate(startPose);


    Trajectory trajectory1 = drive.trajectoryBuilder(startPose, true)
            .splineTo(new Vector2d(traj1x, -traj1y), traj1heading)
            .build();

    Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
            .forward(traj2forward)
            .build();

    Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end().plus(new Pose2d(0.0, 0.0, -traj2turn)))
            .forward(traj3forward - 4.0)
            .build();

    Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
            .back(traj4back)
            .build();

//        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end().plus(new Pose2d(0.0, 0.0, traj4turn)), true)
//                .splineTo(new Vector2d(traj5x, traj5y), traj5heading)
//                .build(); //SEVERE CORRECTION ADDED TO CORRECT RR ERROR
//        //check uncorrected auton doc for original

    Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end().plus(new Pose2d(0.0, 0.0, -traj4turn)), true)
            .back(traj5back)
            .build();

    waitForStart();

    drive.followTrajectory(trajectory1);

    intake.setPower(1.0);
    hold(3.0); //spin ducks
    intake.setPower(0.0);

    drive.followTrajectory(trajectory2);

    drive.turn(-traj2turn);

    drive.followTrajectory(trajectory3);

    intake.setPower(1.0);
    hold(5.0); //deposit preload
    intake.setPower(0.0);

    drive.followTrajectory(trajectory4);

    drive.turn(-traj4turn);

    drive.followTrajectory(trajectory5);

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
