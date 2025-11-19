package org.firstinspires.ftc.teamcode.systemTeleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class HoodTeleop {
    public static double hoodPos = 0;
//     Hood hood;
    @TeleOp(name = "CRServo AprilTag PID Tracking Dashboard", group = "Vision")
    @Config
    public class CameraTeleop extends LinearOpMode {
        Hood hood = new Hood(this);
        @Override
        public void runOpMode() {

            // Initialize dashboard and telemetry
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

            // Start camera streaming to Dashboard

            telemetry.addLine("Waiting for start...");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                hood.setPosition(hoodPos);

            }
        }
    }
}
