package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class OutputtingFreight extends OpMode {
    WebcamName webcamName;
    OpenCvCamera camera;
    FindingFreight2 findingFreight;
    double xlower, ylower, xupper, yupper;
    boolean checkifstop = false;
    int counter = 0;
    MOEBot moebot;

    @Override
    public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "FreightCam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        findingFreight = new FindingFreight2();
        camera.openCameraDevice();
        camera.startStreaming(160, 90
                , OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(findingFreight);

//        webcamName = hardwareMap.get(WebcamName.class, "TSECam");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
//        findingFreight = new FindingFreight();
//        camera.openCameraDevice();
//        camera.startStreaming(160, 90, OpenCvCameraRotation.UPRIGHT);
//        camera.setPipeline(findingFreight);
//        moebot = new MOEBot(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void loop() {

//        while (!checkifstop) {
//
////            ArrayList<Double> asdf;
////            asdf = findingFreight.contareas;
////            int counter2 = 0;
////            for (int i = 0; i < asdf.size(); i++) {
////                telemetry.addData("contourssizes", asdf.get(i));
////                telemetry.addData("counter2", counter2);
////                counter2++;
////            }
////            counter++;
////            checkifstop = findingFreight.checkifstop;
////            xlower = findingFreight.lowerx;
////            ylower = findingFreight.lowery;
////            yupper = findingFreight.uppery;
////            xupper = findingFreight.upperx;
////
////            telemetry.addData("xlower", xlower);
////            telemetry.addData("ylower", ylower);
////            telemetry.addData("yupper", yupper);
////            telemetry.addData("xupper", xupper);
////            telemetry.addData("checkifstop", checkifstop);
////            telemetry.addData("counter", counter);
////
////            moebot.chassis.setLeftPower(0.5, 0.2);
//
//            // have motor powers set to turn
//        }
        telemetry.addData("position", "somethign");
        telemetry.addData("greatestcontour", findingFreight.maxarea);
        telemetry.addData("counter", findingFreight.counter);
        telemetry.addData("area", findingFreight.getXPos());
        telemetry.addData("xpos", findingFreight.getXPos());
        telemetry.addData("position", findingFreight.getPosition());

    }
}