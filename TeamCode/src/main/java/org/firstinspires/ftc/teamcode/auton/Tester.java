//package org.firstinspires.ftc.teamcode.auton;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Autonomous
//public class Tester extends OpMode {
//    WebcamName webcamName;
//    OpenCvCamera camera;
//    TeamMarkerTracker tracker;
//    int position = 0;
//
//    @Override
//    public void init() {
//        webcamName = hardwareMap.get(WebcamName.class, "HighCam");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
//        tracker = new TeamMarkerTracker();
//        camera.openCameraDevice();
//        camera.startStreaming(160, 90, OpenCvCameraRotation.UPRIGHT);
//        camera.setPipeline(tracker);
//    }
//
//    @Override
//    public void loop() {
//        telemetry.addData("position", "somethign");
//        telemetry.addData("counter", tracker.coutner);
//        telemetry.addData("chalnes", tracker.channels);
//        telemetry.addData("area", tracker.area);
//        telemetry.addData("xpos", tracker.xpos);
//        telemetry.addData("position", tracker.position);
//    }
//}