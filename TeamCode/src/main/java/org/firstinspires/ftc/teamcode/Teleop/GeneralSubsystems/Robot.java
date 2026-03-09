package org.firstinspires.ftc.teamcode.Teleop.GeneralSubsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.List;
import org.firstinspires.ftc.teamcode.Teleop.GeneralSubsystems.Limelight;
import org.firstinspires.ftc.teamcode.Teleop.GeneralSubsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {

    public Follower follower;
    public DcMotor fr, fl, br, bl;
//    public IMU imu;
    public static DcMotorEx lfMotor, rfMotor, lbMotor, rbMotor, shooterMotor3, transfer, intakeMotor, shooterMotor2;
    public static Servo doorToucher;

//    public Outtake outtake;
//    public Intake intake;
    public Limelight limelight;


    private final AllianceColor allianceColor;

    public Robot(LinearOpMode opMode) {
        this(opMode, AllianceColor.RED);
    }

    public Robot(LinearOpMode opMode, AllianceColor allianceColor) {
        this.allianceColor = allianceColor;

        HardwareMap hardwareMap = opMode.hardwareMap;
        follower = Constants.createFollower(hardwareMap);

        // From https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        opMode.telemetry.setMsTransmissionInterval(11);

        lfMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        rfMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        lbMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        rbMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");;

        lfMotor.setDirection(DcMotor.Direction.FORWARD);
        rfMotor.setDirection(DcMotor.Direction.REVERSE);
        lbMotor.setDirection(DcMotor.Direction.FORWARD);
        rbMotor.setDirection(DcMotor.Direction.REVERSE);

        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                LogoFacingDirection.RIGHT,
//                UsbFacingDirection.UP));
//        imu.initialize(parameters);
//        opMode.telemetry.addData("IMU Initialized", true);
//        opMode.telemetry.update();

        // Init Subsystems
//        outtake = new Outtake(opMode);
//        intake = new Intake(opMode);

        limelight = new Limelight(opMode, this.getAllianceColor());
    }

//    public void initAuton() {
//        this.outtake.setBase();
//    }

    public AllianceColor getAllianceColor() {
        return this.allianceColor;
    }

    public void updateAutoControls() {
        follower.update();
//        outtake.updatePIDControl();
    }
}