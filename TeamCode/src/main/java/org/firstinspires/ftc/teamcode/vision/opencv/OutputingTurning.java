package org.firstinspires.ftc.teamcode.vision.opencv;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OutputingTurning extends OpMode {
    WebcamName webcamName;
    Boolean checkif = false;
    OpenCvCamera camera;
    FindifTurning findifTurning;
    int x = 0, width = 0;
    @Override
    public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "RingCam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        findifTurning = new FindifTurning();
        camera.openCameraDevice();
        camera.startStreaming(160, 90, OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(findifTurning);
    }

    @Override
    public void loop() {
        x = findifTurning.largestx;
        width = findifTurning.width;
        // if (x > ##) {
        telemetry.addData("largestx", x);
        telemetry.addData("widthofimage", width);

        // do action
        telemetry.addData("checkiftrue, turning or not", checkif);
    }
}

