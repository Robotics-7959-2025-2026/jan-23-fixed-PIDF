package org.firstinspires.ftc.teamcode.Autonomous.Vision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Disabled
@Autonomous(name="Keir April Tag Test February 10th")
public class AprilTagTester extends LinearOpMode{
    final int cameraWidth = 1280;
    final int cameraHeight = 720;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    OttoAprilTagPipeline pipeline = new OttoAprilTagPipeline(tagsize, fx,fy,cx,cy);

    public void runOpMode() throws InterruptedException{
        telemetry.addData("Starting otto now: check camera to init - adyn is a furry", "Ready");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }


            @Override
            public void onError(int errorCode) {


            }
        });






        //view the webcam now, you cant check camera stream once the game is initialized.
        waitForStart();


        telemetry.addData("Succesfully init", "Running");
        telemetry.update();

        while(opModeIsActive()){

            //pit i 45 inch and 23 by firt tile
//            telemetry.addData("Pitch:", ItWasAllRedAndYellow.getAngle());
//            telemetry.addData("Yaw:", ItWasAllRedAndYellow.getCoordsX());
//            telemetry.addData("Roll:", ItWasAllRedAndYellow.getCoordsY());
//            telemetry.addData("Value:", ItWasAllRedAndYellow.getColor());

            //telemetry.addData("Angle:", ItWasAllBlueAndYellow.getAngle());
            //telemetry.addData("CenterX:", ItWasAllBlueAndYellow.getCoordsX());
            //telemetry.addData("CenterY:", ItWasAllBlueAndYellow.getCoordsY());
            //telemetry.addData("Color:", ItWasAllBlueAndYellow.getColor());

            telemetry.update();

        }



        telemetry.addData("Status", "GG");
        telemetry.update();
    }






}
