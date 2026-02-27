package org.firstinspires.ftc.teamcode.Autonomous.FarSide.Opmode;

import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.Teleop.newPIDFController2;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

public abstract class FarAuto extends LinearOpMode {
    public FarPaths paths = null;
    public Follower pedro = null;

    public DcMotorEx intake = null;
    public double intakeDesired = 0;
    public DcMotorEx shooter2 = null;
    public DcMotorEx shooter3 = null;
    public double P = 0.024;
    public double kV = 0.000387;
    public double I = 0;
    public double kS = 0.11;
    public double kD = 0.0;
    public double kF = 0.0;
    public newPIDFController2 flywheelController =
            new newPIDFController2(P, I, kD, kF);

    // Velocity to use when shooting
    public double shooterHigh = 1480;
    // Velocity to use now
    public double shooterTarget = 0;
    public DcMotorEx transfer = null;
    public double transferHigh = 1.0;
    public double transferTarget = 0.0;

    VoltageSensor battery = null;
    double batteryVoltage = 0.0;

    public double nominalVoltage = 12.6;

    public abstract FarPaths getPaths();

    @Override
    public void runOpMode() {
        pedro = Constants.createFollower(hardwareMap);
        paths = getPaths();
        pedro.setStartingPose(paths.startPose);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooter3 = hardwareMap.get(DcMotorEx.class, "shooterMotor3");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter3.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelController.setPIDF(P, I, kD, kF);
        flywheelController.setFeedforward(kV, 0.0, kS);

        intake.setPower(0);
        shooter2.setPower(0);
        shooter3.setPower(0);
        transfer.setPower(0);

        battery = hardwareMap.voltageSensor.iterator().next();
        batteryVoltage = battery.getVoltage();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Started");
        telemetry.update();

        intakeDesired = 1.0;
        shooterTarget = shooterHigh;

        update();
        //far side order will be pre, grab&shoot one, grab&shoot two.
        if (goTo(paths.shootPre)) {
            return;
        }

        update();
        if (waitMillis(1500)) {
            return;
        }
        update();

        // Shoot for 3s
        transferTarget = transferHigh;
        if (waitMillis(2500)) {
            return;
        }

        update();

        transferTarget = 0.0;
        shooterTarget = 0.0;

        // Grab balls 4, 5, 6
        if (getAndShoot(paths.grabOne, paths.shootOne, 2500, 500)) {
            return;
        }

        if(waitMillis(3000)){
            return;
        }
        update();
        // Grab balls 7, 8, 9
        if (getAndShoot(paths.grabTwo, paths.shootTwo, 3000)) {
            return;
        }

        if(goTo(paths.leaveZone)){ return; }

        // Extra wait
        if (waitMillis(200)) {
            return;
        }

        update();

        telemetry.addData("Status", "Done");
    }

    // Always return if this returns true
    public boolean waitMillis(long millis) {
        Timing.Timer timer = new Timing.Timer(millis, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done()) {
            if (isStopRequested()) {
                return true;
            }
            telemetry.addData("Status", "Waiting");
            telemetry.addData("Wait progress", timer.elapsedTime());
            update();
        }
        telemetry.addData("Status", "Running");
        return false;
    }

    // Always return if this returns true
    public boolean goTo(PathChain loc) {
        pedro.followPath(loc);
        while (pedro.isBusy()) {
            if (isStopRequested()) {
                return true;
            }
            telemetry.addData("Status", "Moving");
            update();
        }
        telemetry.addData("Status", "Running");
        return false;
    }

    public void update() {
        telemetry.update();
        batteryVoltage = battery.getVoltage();

        pedro.update();

        double cVel = shooter2.getVelocity();
        telemetry.addData("TargetVel", shooterTarget);
        telemetry.addData("CurrentVel", cVel);
        flywheelController.setPIDF(P, I, kD, kF);
        flywheelController.setFeedforward(kV, 0.0, kS);
        shooter2.setPower(flywheelController.calculate(shooterTarget - cVel, shooterTarget, 0.0));
        shooter3.setPower(flywheelController.calculate(shooterTarget - cVel, shooterTarget, 0.0));

        transfer.setPower(transferTarget);

        double corrected = intakeDesired * (nominalVoltage / Math.max(1.0, batteryVoltage));
        corrected = Math.max(-1.0, Math.min(1.0, corrected));
        intake.setPower(corrected);
    }

    //
    public boolean getAndShoot(PathChain get, PathChain shoot) {
        intakeDesired = 1.0;
        if (goTo(get)) {
            return true;
        }

        update();

        shooterTarget = shooterHigh;
        if (goTo(shoot)) {
            return true;
        }

        update();


        transferTarget = transferHigh;
        if (waitMillis(3000)) {
            return true;
        }

        update();


        intakeDesired = 0.0;
        transferTarget = 0.0;
        shooterTarget = 0.0;
        update();

        return false;
    }

    //this is awful but its called method overloading in java, its a common practice so chill on me
    public boolean getAndShoot(PathChain get, PathChain shoot, int wait) {
        intakeDesired = 1.0;
        if (goTo(get)) {
            return true;
        }
        //waits for intaking
        if(waitMillis(wait)){
            return true;
        }
        update();

        shooterTarget = shooterHigh;
        //shooter is on high as it goes to shoot

        if (goTo(shoot)) {
            return true;
        }

        update();


        transferTarget = transferHigh;
        if (waitMillis(3000)) {
            return true;
        }

        update();


        intakeDesired = 0.0;
        transferTarget = 0.0;
        shooterTarget = 0.0;
        update();

        return false;
    }

    public boolean getAndShoot(PathChain get, PathChain shoot, int wait, int shooterWait) {
        intakeDesired = 1.0;
        if (goTo(get)) {
            return true;
        }
        //waits for intaking
        if(waitMillis(wait)){
            return true;
        }
        update();

        shooterTarget = shooterHigh;
        //shooter is on high as it goes to shoot
        update();
        if(waitMillis(shooterWait)){
            return true;
        }
        update();
        if (goTo(shoot)) {
            return true;
        }

        update();


        transferTarget = transferHigh;
        if (waitMillis(3000)) {
            return true;
        }

        update();


        intakeDesired = 0.0;
        transferTarget = 0.0;
        shooterTarget = 0.0;
        update();

        return false;
    }
}
