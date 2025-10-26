package org.firstinspires.ftc.teamcode.systemTeleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "CRServo AprilTag Tracking Dashboard", group = "Vision")
@Config
public class CameraTeleop extends LinearOpMode {

    // Dashboard-tunable PD constants
    public static double KP = 0.015;
    public static double KD = 0.004;
    public static double TOLERANCE = 1.0; // degrees
    public static int TARGET_TAG_ID = 0;

    private CRServo turretServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private double lastError = 0;
    private double lastTime = 0;

    @Override
    public void runOpMode() {
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turretServo.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create Vision Portal with dashboard camera stream
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Initialize dashboard and telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Start camera streaming to Dashboard
        dashboard.startCameraStream(visionPortal, 0);

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    if (detection.id == TARGET_TAG_ID) {
                        double error = detection.ftcPose.bearing;

                        // PD Control
                        double currentTime = getRuntime();
                        double dt = currentTime - lastTime;
                        double derivative = (error - lastError) / (dt > 0 ? dt : 1);
                        double power =  KP * error + KD * derivative;

                        // Clamp output
                        power = Math.max(-1, Math.min(1, power));

                        turretServo.setPower(power);

                        lastError = error;
                        lastTime = currentTime;

                        telemetry.addData("Tag ID", detection.id);
                        telemetry.addData("Bearing (deg)", "%.2f", error);
                        telemetry.addData("Servo Power", "%.3f", power);
                        telemetry.addData("Distance (m)", "%.2f", detection.ftcPose.range);
                        break;
                    }
                }
            } else {
                turretServo.setPower(0);
                telemetry.addLine("No AprilTag detected");
            }

            telemetry.update();
        }

        turretServo.setPower(0);
        visionPortal.close();
    }
}
