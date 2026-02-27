package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Opmode;

import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Teleop.newPIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

public abstract class ManualAuto extends LinearOpMode {
    public Paths paths = null;
    public Follower pedro = null;

    public DcMotorEx intake = null;
    /** The current <b>power ratio</b> of the transfer motor
     */
    public double intakeDesired = 0;
    
    public long shootDuration = 1000;

    public DcMotorEx shooter2 = null;
    public DcMotorEx shooter3 = null;

    /** PID factor for shooters
     */
    public double P = 0.015;
    /** PID factor for shooters
     */
    public double kV = 0.00036;
    /** PID factor for shooters
     */
    public double I = 0;
    /** PID factor for shooters
     */
    public double kS = 0.065067;
    /** PID factor for shooters
     */
    public double kD = 0.0;
    /** PID factor for shooters
     */
    public double kF = 0.0;
    /** Specifically for shooters
     */
    public newPIDFController flywheelController =
            new newPIDFController(P, I, kD, kF);

    public double shooterHigh = 1170;
    /** The current <b>velocity</b> of the shooter motors
     */
    public double shooterTarget = 0;

    public DcMotorEx transfer = null;
    public double transferHigh = 1.0;
    /** The current <b>power ratio</b> of the transfer motor
     */
    public double transferTarget = 0.0;

    VoltageSensor battery = null;
    double batteryVoltage = 0.0;

    public double nominalVoltage = 12.6;

    /** Obtain the Paths for this instance
     * @return The set of Paths to be followed throughout the auton
     * @see Paths
     */
    public abstract Paths getPaths();

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

        intakeDesired = 0.0;
        shooterTarget = shooterHigh;
        transferTarget = 0.0;
        //new order will be pre, grab two, hit lever, shootTwo, grabOne, shootOne, grabThree, shootThree, leaveZone
        if (goTo(paths.shootPre)) {
            return;
        }

        if (waitMillis(500)) {
            return;
        }

        update();

        // Shoot for 3s

        transferTarget = transferHigh;
        intakeDesired = 1.0;
        if (waitMillis(shootDuration)) {
            return;
        }

        update();

        transferTarget = 0.0;
        shooterTarget = 0.0;

        intakeDesired = 1.0;
        update();

        if(getAndShootBetter(paths.grabTwo, paths.shootTwo)){
            return;
        }
        update();

        // Grab balls 4, 5, 6
        if (getAndShootBetter(paths.grabOne, paths.shootOne)) {
            return;
        }

        update();
        // Grab balls 10, 11, 12
        if (getAndShootBetter(paths.grabThree, paths.shootThree)) {
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

    /** Idle for a duration, but keep running update
     *
     * @param millis the number of ms to wait
     * @return true if early exit requested
    */
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

    /** Follow a path until completion
     *
     * @see PathChain
     * @param loc the PedroPathing PathChain to follow
     * @return true if early exit requested
     */
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

    /** Update shooter/transfer/intake motors and PIDs
     * <p>
     * You <b>MUST</b> run this to update the motors, or else the <i>will not update</i>
     */
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

    /** Handles acquiring and shooting artifacts based on PedroPaths
     *
     * @param get The path to get the artifacts
     * @param shoot The path to follow before shooting the artifacts
     * @return true if early exit requested
     * @see PathChain
     */
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
        if (waitMillis(shootDuration)) {
            return true;
        }

        update();


        intakeDesired = 0.0;
        transferTarget = 0.0;
        shooterTarget = 0.0;
        update();

        return false;
    }

    /** Handles acquiring and shooting artifacts based on PedroPaths, but better
     *
     * @param get The path to get the artifacts
     * @param shoot The path to follow before shooting the artifacts
     * @return true if early exit requested
     * @see PathChain
     */
    public boolean getAndShootBetter(PathChain get, PathChain shoot) {
        intakeDesired = 1.0;
        if (goTo(get)) {
            return true;
        }

        intakeDesired = 0.0;
        shooterTarget = shooterHigh;

        update();

        if (goTo(shoot)) {
            return true;
        }

        update();

        intakeDesired = 1.0;
        transferTarget = transferHigh;
        if (waitMillis(shootDuration)) {
            return true;
        }

        update();


        intakeDesired = 0.0;
        transferTarget = 0.0;
        shooterTarget = 0.0;
        update();

        return false;
    }

    /** Handles acquiring and shooting artifacts based on PedroPaths
     *
     * @param get The path to get the artifacts
     * @param shoot The path to follow before shooting the artifacts
     * @param intakeDelay The ms delay to wait after intaking
     * @return true if early exit requested
     * @see PathChain
     */
    public boolean getAndShoot(PathChain get, PathChain shoot, long intakeDelay) {
        intakeDesired = 1.0;
        if (goTo(get)) {
            return true;
        }

        update();

        if (waitMillis(intakeDelay)) {
            return true;
        }

        update();

        shooterTarget = shooterHigh;
        if (goTo(shoot)) {
            return true;
        }

        update();


        transferTarget = transferHigh;
        if (waitMillis(shootDuration)) {
            return true;
        }

        update();


        intakeDesired = 0.0;
        transferTarget = 0.0;
        shooterTarget = 0.0;
        update();

        return false;
    }

    /** Handles acquiring and shooting artifacts based on PedroPaths
     *
     * @param get The path to get the artifacts
     * @param shoot The path to follow before shooting the artifacts
     * @param intakeDelay The ms delay to wait after intaking
     * @param shooterDelay The ms delay to wait before going to shoot, after
     * @return true if early exit requested
     * @see PathChain
     */
    public boolean getAndShoot(PathChain get, PathChain shoot, long intakeDelay, long shooterDelay) {
        intakeDesired = 1.0;
        if (goTo(get)) {
            return true;
        }

        update();

        if (waitMillis(intakeDelay)) {
            return true;
        }

        update();

        shooterTarget = shooterHigh;
        if (waitMillis(shooterDelay)) {
            return true;
        }

        if (goTo(shoot)) {
            return true;
        }

        update();


        transferTarget = transferHigh;
        if (waitMillis(shootDuration)) {
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
