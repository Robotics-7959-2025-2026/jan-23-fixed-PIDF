package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Teleop.Motors.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// import org.firstinspires.ftc.teamcode.Autonomous.Vision.AprilTagCamera;


//We are so back
@Configurable
@TeleOp(name = "7959 Teleop Blue")
public class OfficialTeleopBlue extends LinearOpMode {
    public newPIDFController flywheelController =
            new newPIDFController(0.015, 0.0, 0.065067, 0.0);

    private Limelight3A limelight;


    private double nominalVoltage = 12.5;
    public static double targetVelocity, velocity;
    private double desiredPower = 1;
    private double batteryVoltage, correctedPower;
    double kV, I, kS;
    double currentVelocity;
    double mf, ms, mr;
    private double ctrlPow = 2.0;
    double error;

    double curVelocity;
    double curTargetVelocity = 1250;
    double farTargetVelocity = 2000;
    private static final double BASE_F = 14.5;
    private static final double BASE_P = 0.4;
    double F = 14.02;
    double P = 0.015;
    // private AprilTagCamera ATC = null;
    // private AprilTagDetection goalTag;
    // private List<AprilTagDetection> tagDetections = new ArrayList<>();
    private double aimbotP = 0.0296;
    private double aimbotError = 0.0;
    private double aimbotD = 0.001;
    private double aimbotPrevErr = 0.0;
    private double goalX = 3.0;
    double aimbotAngleTolerance = 0.01;
    double curTime = 0;
    double lastTime = 0;


    @Override
    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        Motors.init(this.hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        /*
         * Starts polling for data.
         */
        limelight.start();




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
        // ATC = new AprilTagCamera(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            batteryVoltage = battery.getVoltage();
            LLResult result = limelight.getLatestResult();

            double distance = 0.0;

            if (result != null && result.isValid()) {

                double ty = result.getTy();

                double cameraHeight = 0.42;   // meters (measure yours)
                double targetHeight = 0.7493;   // meters (game dependent)
                double cameraAngle = Math.toRadians(0); // mounting angle

                distance =
                        (targetHeight - cameraHeight) /
                                Math.tan(cameraAngle + Math.toRadians(ty));

                telemetry.addData("Distance (m)", distance);

                telemetry.addData("Distance (m)", distance);
            }
            if(result != null){
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
            }

            mf = gamepad1.left_stick_y;
            ms = gamepad1.left_stick_x;
            mr = gamepad1.right_stick_x * 0.75;






//
//
//            ATC.update();
//            tagDetections = ATC.getTagDetections();
//            if (!tagDetections.isEmpty()) {
//                goalTag = null;
//
//                // Filter for decent tags
//                for (AprilTagDetection gT : tagDetections) {
//                    if (gT.id == 20 || gT.id == 24) {
//                        goalTag = gT;
//                        break;
//                    }
//                }
//
//                if (goalTag != null) {
//                    telemetry.addData("Distance in cm", "%.2f", goalTag.ftcPose.range);
//                } else {
//                    telemetry.addData("GoalTag", "==null");
//                }
//            }

            if (gamepad1.right_stick_button && result != null && result.isValid()) {

                double tx = result.getTx(); // horizontal error

                double cameraOffset = 0.1252; // meters from robot center
                double offsetAngle = Math.toDegrees(Math.atan(cameraOffset / distance));

                error = tx + offsetAngle;


                double pTerm = error * aimbotP;

                curTime = getRuntime();
                double dT = curTime - lastTime;

                double dTerm = 0;
                if (dT > 0.001) {
                    dTerm = ((error - aimbotPrevErr) / dT) * aimbotD;
                }

                mr += Range.clip(pTerm + dTerm, -0.5, 0.5);

                aimbotPrevErr = error;
                lastTime = curTime;
                double distanceCM = distance * 100;

                targetVelocity = (232.0116 * distance) + 778.90026;
                curTargetVelocity = targetVelocity;
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

            currentVelocity = shooterMotor3.getVelocity();
            kV = 0.00036;
            I = 0;
            kS = 0.065067;
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

            if (gamepad1.bWasPressed()) {

                curTargetVelocity += 50.0;

            }

            if (gamepad1.xWasPressed()){

                curTargetVelocity -= 50.0;

            }
            curVelocity = shooterMotor3.getVelocity();
            double shooterError = curTargetVelocity - curVelocity;

            telemetry.addData("Target velocity", curTargetVelocity);
            telemetry.addData("Current Velocity", "%.2f", curVelocity);
            telemetry.addData("Error", "%.2f", error);

            telemetry.update();

        }
    }
}