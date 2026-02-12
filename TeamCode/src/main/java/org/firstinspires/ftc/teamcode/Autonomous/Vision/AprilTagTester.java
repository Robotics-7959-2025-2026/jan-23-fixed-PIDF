package org.firstinspires.ftc.teamcode.Autonomous.Vision;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp(name="Keir April Tag Test February 10th")
public class AprilTagTester extends LinearOpMode{
    final int cameraWidth = 1280;
    final int cameraHeight = 720;
    double yaw;
    ArrayList<AprilTagDetection> currentDetections = new ArrayList<>();
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive = false;



    double x,y, pitchError;
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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
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
        follower.startTeleOpDrive();

        while(opModeIsActive()){
            follower.update();

            if(!automatedDrive){
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true
                );
            }


            currentDetections = OttoAprilTagPipeline.getLatestDetections();
            if(!currentDetections.isEmpty()){
                telemetry.addData("AprilTag Detections", OttoAprilTagPipeline.getLatestDetections());
                x = currentDetections.get(0).center.x;
                y = currentDetections.get(0).center.y;

                Orientation rot = Orientation.getOrientation(currentDetections.get(0).pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);
                //yaw
                currentDetections.get(0).pose.z = -rot.firstAngle;
                yaw = currentDetections.get(0).pose.z;

                currentDetections.get(0).pose.y = -rot.thirdAngle;
                //pitch
                currentDetections.get(0).pose.x = -rot.secondAngle;

//                Path adjustToAprilTag = new Path(new BezierLine(follower::getPose, new Pose(45, 98)))
                if(gamepad1.dpadDownWasPressed()){
                    Path adjustToAprilTag = new Path(new BezierLine(follower::getPose, new Pose(follower.getPose().getX()-yaw, follower.getPose().getX()-yaw)));
                    follower.followPath(adjustToAprilTag);
                    automatedDrive = true;
                }
                //pitch is x, roll is y and yaw is z.
                telemetry.addData("Follower busy?", follower.isBusy());
                telemetry.addData("robot pose", follower.getPose());
                telemetry.addData("yaw", -rot.firstAngle);
                telemetry.addData("pitch", -rot.secondAngle);
                telemetry.addData("roll", -rot.thirdAngle);

                telemetry.update();
            }else{
                telemetry.addData("No April tag detections", "son");
                telemetry.update();
            }

            if (automatedDrive && (gamepad1.dpadDownWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }
            telemetry.update();

        }


        telemetry.addData("Status", "GG");
        telemetry.addData("position", follower.getPose());
        telemetry.update();
    }






}