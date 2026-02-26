package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Teleop.Motors.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Autonomous.Vision.AprilTagCamera;
import org.firstinspires.ftc.teamcode.Teleop.newPIDFController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

//We are so back
@Configurable
@TeleOp(name = "7959 Teleop")
public class OfficialTeleop extends LinearOpMode {
    public newPIDFController flywheelController =
            new newPIDFController(0.015, 0.0, 0.065067, 0.0);


    private double nominalVoltage = 12.5;
    public static double targetVelocity, velocity;
    private double desiredPower = 1;
    private double batteryVoltage, correctedPower;

    private double ctrlPow = 2.0;
    double error;
    double curVelocity;
    double curTargetVelocity = 1250;
    double farTargetVelocity = 2000;
    private static final double BASE_F = 14.5;
    private static final double BASE_P = 0.4;
    double F = 14.02;
    double P = 0.015;
    private boolean doorToggle = false;
    private AprilTagCamera ATC = null;
    private AprilTagDetection goalTag;
    private List<AprilTagDetection> tagDetections = new ArrayList<>();
    private double aimbotP = 0.0296;
    private double aimbotError = 0.0;
    private double aimbotD = 0.001;
    private double aimbotPrevErr = 0.0;
    private double goalX = 4.0;
    double aimbotAngleTolerance = 0.01;
    double curTime = 0;
    double lastTime = 0;

    @Override
    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        Motors.init(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        batteryVoltage = battery.getVoltage();
//        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P, 0, 0, F);
//        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
//        shooterMotor3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelController.setFeedforward(
                0.00036, // kV
                0.0,    // kA (not needed for flywheel)
                0.065067     // kS
        );
        ATC = new AprilTagCamera(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            batteryVoltage = battery.getVoltage();

            double mf = gamepad1.left_stick_y;
            double ms = gamepad1.left_stick_x;
            double mr = gamepad1.right_stick_x * 0.75;

            ATC.update();
            //can align using specific id, for now its just gonna align to the one
            //that it sees
            tagDetections = ATC.getTagDetections();
            if (!tagDetections.isEmpty()) {
                goalTag = tagDetections.get(0);
                telemetry.addData("Distance in cm", "%.2f", goalTag.ftcPose.range);
            }

            if (gamepad1.right_stick_button) {
                if(goalTag != null){
                    error = goalX-goalTag.ftcPose.bearing; //angle away from target

                    if (Math.abs(error) >= aimbotAngleTolerance) {
                        double pTerm = error * aimbotP;

                        curTime = getRuntime();
                        double dT = curTime-lastTime;
                        double dTerm = ((error-aimbotPrevErr)/ dT) * aimbotD;

                        mr += Range.clip(pTerm + dTerm, -0.5, 0.5);

                        //targetVelocity = ((-0.0119164 * goalTag.ftcPose.range)*(-0.0119164 * goalTag.ftcPose.range)) + (8.25141 * (goalTag.ftcPose.range)) + 710.09773;
                        targetVelocity = (6.7762 * (goalTag.ftcPose.range)) + 751.44476;
                        curTargetVelocity = targetVelocity;
                        aimbotPrevErr = error;
                        lastTime = curTime;
                    }
                } else {
                    lastTime = getRuntime();
                    aimbotPrevErr = 0;
                }
            } else {
                aimbotPrevErr = 0;
                lastTime = getRuntime();
            }

            rfMotor.setPower(mf + ms + mr);
            lfMotor.setPower(mf - ms - mr);
            rbMotor.setPower(mf - ms + mr);
            lbMotor.setPower(mf + ms - mr);

            //hold left bumper to spin, then press the right bumper to shoot
            if (gamepad1.left_bumper) {
                correctedPower = desiredPower * (nominalVoltage / Math.max(batteryVoltage, 1.0));
                correctedPower = Math.max(-1.0, Math.min(correctedPower, 1.0));
                intakeMotor.setPower(correctedPower);
            } else {
                intakeMotor.setPower(0.0);
            }

            double currentVelocity = shooterMotor3.getVelocity();
            double kV = 0.00036;
            double I = 0;
            double kS = 0.065067;
            telemetry.addData("TargetVel", targetVelocity);
            telemetry.addData("CurrentVel", velocity);
            flywheelController.setPIDF(P,I, 0.0, 0);
            flywheelController.setFeedforward(kV,0,kS);
            velocity = shooterMotor2.getVelocity();

            // ===== CUSTOM FLYWHEEL VELOCITY CONTROL =====

            if (gamepad1.right_bumper) {
                targetVelocity = curTargetVelocity;
                shooterMotor2.setPower(flywheelController.calculate(targetVelocity - velocity,targetVelocity,0));
                shooterMotor3.setPower(flywheelController.calculate(targetVelocity - velocity,targetVelocity,0));

            } else {
                targetVelocity = 0;
                shooterMotor2.setPower(0.0);
                shooterMotor3.setPower(0.0);
            }
            if(gamepad1.left_trigger>0.2){
                transfer.setPower(0.7);
            }else{
                transfer.setPower(0);
            }



            if (currentGamepad1.x && !previousGamepad1.x) {
                doorToggle = !doorToggle;
            }

            if(gamepad1.x){
                if (doorToggle) {
                    doorToucher.setPosition(0.3);
                }else{
                    doorToucher.setPosition(0);
                }
            }

            if (gamepad1.bWasPressed()){

                curTargetVelocity += 50.0;

            }

            if (gamepad1.xWasPressed()){

                curTargetVelocity -= 50.0;

            }
            curVelocity = shooterMotor3.getVelocity();
            error = curTargetVelocity + curVelocity;

            telemetry.addData("Target velocity", curTargetVelocity);
            telemetry.addData("Current Velocity", "%.2f", curVelocity);
            telemetry.addData("Error", "%.2f", error);

            telemetry.update();

        }
    }
}