//package org.firstinspires.ftc.teamcode.systemTeleops;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Utils.PID;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//@TeleOp(name = "CRServo AprilTag PID Tracking Dashboard", group = "Vision")
//@Config
//@Disabled
//public class CameraTeleop extends LinearOpMode {
//
//    // Dashboard-tunable PID constants
//    public static double KP = 0.0133;
//    public static double KI = 0.0001;      // <-- new integral gain (start small!)
//    public static double KD = 0.0000001;
//
//    public static int TARGET_TAG_ID = 20;
//
//    // Optional tuning parameters
//    public static double MAX_POWER = 0.6;
//    public static double INTEGRAL_LIMIT = 100.0; // anti-windup limit
//    public static double DEADZONE = 0.3; // degrees
//
//    private CRServo turretServo;
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//
//    private double lastError = 0;
//    private double lastTime = 0;
//    private double integralSum = 0;
//
//    @Override
//    public void runOpMode() {
//        PID pid;
//        pid = new PID(KP, KI, KD, MAX_POWER, INTEGRAL_LIMIT, DEADZONE, this);
//        turretServo = hardwareMap.get(CRServo.class, "turretServo");
//
//        // Initialize AprilTag processor
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        // Create Vision Portal with dashboard camera stream
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .build();
//
//        // Initialize dashboard and telemetry
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//
//        // Start camera streaming to Dashboard
//        dashboard.startCameraStream(visionPortal, 0);
//
//        telemetry.addLine("Waiting for start...");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            List<AprilTagDetection> detections = aprilTag.getDetections();
//
//            if (!detections.isEmpty()) {
//                for (AprilTagDetection detection : detections) {
//                    if (detection.id == TARGET_TAG_ID) {
//                        break;
//                    }
//                }
//            } else {
//                // No tag detected → stop servo + reset integral to prevent drift
//
//                turretServo.setPower(0);
//                integralSum = 0;
//                telemetry.addLine("No AprilTag detected");
//            }
//
//            telemetry.update();
//        }
//
//        turretServo.setPower(0);
//        visionPortal.close();
//    }
//}
