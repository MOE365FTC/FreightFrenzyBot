package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    WebcamName webcamName;
    OpenCvCamera camera;
    public TeamMarkerTracker TSETracker;

    //Teleop Constructor
    public MOEBot(HardwareMap hardwareMap, Gamepad gpad1, Gamepad gpad2){
        chassis = new Chassis(hardwareMap, gpad1);
        slides = new Slides(hardwareMap, gpad2);
        intake = new Intake(hardwareMap, gpad2, this.slides);
        dispenser = new Dispenser(hardwareMap, gpad2, this.slides);
        carousel = new Carousel(hardwareMap, gpad1);
    }

    //Autonomous Constructor
    public MOEBot(HardwareMap hardwareMap, Gamepad gpad1, Gamepad gpad2, LinearOpMode opMode, int headingOffset){
        chassis = new Chassis(hardwareMap, gpad1, opMode, headingOffset);
        slides = new Slides(hardwareMap, gpad2, opMode);
        intake = new Intake(hardwareMap, gpad2, this.slides);
        dispenser = new Dispenser(hardwareMap, gpad2, this.slides);

        TSETracker = new TeamMarkerTracker();
        webcamName = hardwareMap.get(WebcamName.class, "TSECam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        TSETracker = new TeamMarkerTracker();
        camera.openCameraDevice();
        camera.startStreaming(160, 90
                , OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(TSETracker);
    }
}
