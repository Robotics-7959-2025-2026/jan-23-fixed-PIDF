package org.firstinspires.ftc.teamcode.Teleop.TuningFixes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Teleop.newPIDFController;
import org.firstinspires.ftc.teamcode.Teleop.newPIDFController2;

@Configurable
@TeleOp
public class hunterthugtuningfar extends OpMode {

    private newPIDFController2 controller;
    private DcMotorEx motor;
    private DcMotorEx motor2;
    public static double targetVelocity, velocity;
    public static double P,I,kV,kS;

    // p = 0.015
    // I = 0
    // kS = 0.11
    // kV = 0.000387
    // targetvelocity = 1480


    @Override
    public void init() {
        //TODO: Set motor name and direction
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor3");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        controller = new newPIDFController2(P,I,0.0, 0.0); // PIDF, Feedforward (kV, kA, kS)
    }

    @Override
    public void loop() {
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("CurrentVel", velocity);
        controller.setPIDF(P,I, 0.0, 0);
        controller.setFeedforward(
                0.000395, // kV
                0.0,    // kA (not needed for flywheel)
                0.11    // kS
        );
        velocity = motor.getVelocity();
        motor.setPower(controller.calculate(targetVelocity - velocity,targetVelocity,0));
        motor2.setPower(controller.calculate(targetVelocity - velocity,targetVelocity,0));
    }
}