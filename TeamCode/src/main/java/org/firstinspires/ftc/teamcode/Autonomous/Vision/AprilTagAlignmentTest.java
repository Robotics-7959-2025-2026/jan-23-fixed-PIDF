package org.firstinspires.ftc.teamcode.Autonomous.Vision;


import static org.firstinspires.ftc.teamcode.Teleop.Motors.lbMotor;
import static org.firstinspires.ftc.teamcode.Teleop.Motors.lfMotor;
import static org.firstinspires.ftc.teamcode.Teleop.Motors.rbMotor;
import static org.firstinspires.ftc.teamcode.Teleop.Motors.rfMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Teleop.Motors;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="BP Alignment test mog")
public class AprilTagAlignmentTest extends LinearOpMode{
    final int cameraWidth = 1280;
    final int cameraHeight = 720;
    double maxSpeed = 1;
    AprilTagCamera ATC;
    AprilTagDetection goalTag;
    private List<AprilTagDetection> tagDetections = new ArrayList<>();
    //power contant
    double kP = 0.002;
    double error = 0;
    //derivative control
    double kD = 0.0001;
    double lastError = 0;
    //derivative offset ong
    double goalX = 0.1;

    //less accurate the webcam, the higher the tolerance needed
    double angleTolerance = 0.2;
    double curTime = 0;
    double lastTime = 0;
    double forward, strafe,rotate;
    double FLP, BLP, FRP, BRP;
    double[] stepSizes = {1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 2;

    public void runOpMode() throws InterruptedException{

        AprilTagCamera ATC = new AprilTagCamera(hardwareMap, telemetry);
        Motors.init(hardwareMap);

        telemetry.addData("Clavicular frame mogged", "by asu frat leader");
        telemetry.update();

        //view the webcam now, you cant check camera stream once the game is initialized.
        waitForStart();
        resetRuntime();
        curTime = getRuntime();

        while(opModeIsActive()) {
            forward = gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            ATC.update();
            //can align using specific id, for now its just gonna align to the one
            //that it sees
            tagDetections = ATC.getTagDetections();
            if(!tagDetections.isEmpty()){
                goalTag = tagDetections.get(0);
            }

            if(gamepad1.left_trigger > 0.3){
                if(goalTag != null){
                    error = goalX-goalTag.ftcPose.bearing; //angle away from target

                    if(Math.abs(error) < angleTolerance){
                        rotate = 0;
                    }else{
                        double pTerm = error * kP;

                        curTime = getRuntime();
                        double dT = curTime-lastTime;
                        double dTerm = ((error-lastError)/ dT) * kD;

                        rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                        lastError = error;
                        lastTime = curTime;
                    }
                }else{
                    lastTime = getRuntime();
                    lastError = 0;
                }
            } else{
                lastError = 0;
                lastTime = getRuntime();
            }

            // drive our motors
            FLP = forward + strafe + rotate;
            BLP = forward - strafe + rotate;
            FRP = forward - strafe - rotate;
            BRP = forward + strafe - rotate;
            lfMotor.setPower(FLP);
            lbMotor.setPower(BLP);
            rfMotor.setPower(FRP);
            rbMotor.setPower(BRP);

            if(gamepad1.bWasPressed()){
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if(gamepad1.dpadRightWasPressed()){
                kP += stepSizes[stepIndex];
            }
            if(gamepad1.dpadLeftWasPressed()){
                kP -= stepSizes[stepIndex];
            }

            if(gamepad1.dpadUpWasPressed()){
                kD += stepSizes[stepIndex];
            }
            if(gamepad1.dpadDownWasPressed()){
                kD -= stepSizes[stepIndex];
            }

            if(goalTag != null){
                if(gamepad1.left_trigger > 0.3){
                    telemetry.addLine("Auto align");
                }
                ATC.displayDetectionTelemetryFor(goalTag);
                telemetry.addData("Tag is:", "%.4f cm away", goalTag.ftcPose.range);
                telemetry.addData("Error", "%.2f", error);
            }else{
                telemetry.addLine("MANUAL ROTATE MODE");
            }

            telemetry.addLine("-----------------------------");
            telemetry.addData("Tuning P", "%.4f (D pad L/R", kP);
            telemetry.addData("Tuning D", "%.4f (D pad U/D", kD);
            telemetry.addData("Step Sizes", "%.4f (B button)", stepSizes[stepIndex]);
            telemetry.update();


        }
    }






}