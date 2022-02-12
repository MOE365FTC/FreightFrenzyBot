package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.enums.TSEArmPos;
import org.firstinspires.ftc.teamcode.teleop.TeamMarkerTracker;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class MOEBot {
    public Chassis chassis;
    public Slides slides;
    public Intake intake;
    public Dispenser dispenser;
    public Carousel carousel;
    public TSEArm tseArm;
    public IMU imu;

    WebcamName webcamName;
    OpenCvCamera camera;
    public TeamMarkerTracker TSETracker;

    //Teleop Constructor
    public MOEBot(HardwareMap hardwareMap, Gamepad gpad1, Gamepad gpad2){
        chassis = new Chassis(hardwareMap, gpad1);
        slides = new Slides(hardwareMap, gpad2);
        dispenser = new Dispenser(hardwareMap, gpad2, this.slides);
        intake = new Intake(hardwareMap, gpad2, this.slides, this.dispenser);
        carousel = new Carousel(hardwareMap, gpad1);
        tseArm = new TSEArm(hardwareMap, gpad1);
        imu = new IMU(hardwareMap);
    }

    //Autonomous Constructor
    public MOEBot(HardwareMap hardwareMap, Gamepad gpad1, Gamepad gpad2, LinearOpMode opMode, int headingOffset){
        String status = "INITIALIZING...";
        opMode.telemetry.addData("status", status);
        opMode.telemetry.update();
        chassis = new Chassis(hardwareMap, gpad1, opMode, headingOffset);
        slides = new Slides(hardwareMap, gpad2, opMode);
        dispenser = new Dispenser(hardwareMap, gpad2, this.slides);
        intake = new Intake(hardwareMap, gpad2, this.slides, this.dispenser);
        tseArm = new TSEArm(hardwareMap, gpad1);
        imu = new IMU(hardwareMap, opMode);

        TSETracker = new TeamMarkerTracker();
        webcamName = hardwareMap.get(WebcamName.class, "TSECam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        TSETracker = new TeamMarkerTracker();
        camera.openCameraDevice();
        camera.startStreaming(160, 90
                , OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(TSETracker);
        status = "READY!";
        opMode.telemetry.addData("status", status);
        opMode.telemetry.update();

    }
}
