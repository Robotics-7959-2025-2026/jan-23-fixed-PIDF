package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import android.annotation.SuppressLint;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

// Basically ripped this from Brogan Pratt
public class AprilTagCamera {
    private AprilTagProcessor atp = null;
    private VisionPortal vp = null;
    private List<AprilTagDetection> tagDetections = new ArrayList<>();

    private Telemetry telemetry;

    public AprilTagCamera(WebcamName camera, Telemetry telemetry) {
        this.telemetry = telemetry;

        atp = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setNumThreads(1)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        vp = new VisionPortal.Builder()
                .addProcessor(atp)
                .setCamera(camera)
                .setCameraResolution(new Size(1280, 720))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        setManualExposure(6, 240);
    }

    private void setManualExposure(int ms, int gain) {
        telemetry.addData("Camera status", "Waiting...");
        telemetry.update();
        while (vp.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException ignored) {

            }
        }

        try {
            ExposureControl ec = vp.getCameraControl(ExposureControl.class);
            GainControl gc = vp.getCameraControl(GainControl.class);

            if (ec.getMode() != ExposureControl.Mode.Manual) {
                ec.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }

            ec.setExposure(ms, TimeUnit.MILLISECONDS);
            Thread.sleep(20);
            gc.setGain(gain);
            Thread.sleep(20);
        } catch (Exception e) {
            telemetry.addData("Camera", "Error: " + e.getMessage());
            telemetry.update();
        }
    }

    public void update() {
        tagDetections = atp.getDetections();
    }

    public List<AprilTagDetection> getTagDetections() {
        return tagDetections;
    }

    // idfc bout no locale, we in the God Blessed US of A
    @SuppressLint("DefaultLocale")
    public void displayDetectionTelemetryFor(AprilTagDetection detection) {
        if (detection == null) return;
        if (detection.metadata != null) {
            telemetry.addLine(String.format(
                    "ID: %d | Name: %s",
                    detection.id,
                    detection.metadata.name
            ));
            telemetry.addLine(String.format(
                    "XYZ (in): %6.1f/%6.1f/%6.1f",
                    detection.ftcPose.x,
                    detection.ftcPose.y,
                    detection.ftcPose.z
            ));
            telemetry.addLine(String.format(
                    "Pitch: %6.1f | Roll: %6.1f | Yaw: %6.1f",
                    detection.ftcPose.pitch,
                    detection.ftcPose.roll,
                    detection.ftcPose.yaw
            ));
        } else {
            telemetry.addLine(String.format("ID: %d has no metadata", detection.id));
        }
    }

    public AprilTagDetection getDetectionForId(int id) {
        for (AprilTagDetection atd : tagDetections) {
            if (atd.id == id) {
                return atd;
            }
        }
        return null;
    }

    public void stop() {
        if (vp != null) {
            vp.close();
        }
    }
}
