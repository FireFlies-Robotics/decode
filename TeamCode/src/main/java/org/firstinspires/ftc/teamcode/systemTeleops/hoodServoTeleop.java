package org.firstinspires.ftc.teamcode.systemTeleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "SERVO", group = "Vision")
@Config
public class hoodServoTeleop extends LinearOpMode {

    private CRServo servo;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(CRServo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.left_trigger > 0.5) {
                servo.setPower(1);
            }
            else if (gamepad1.right_trigger > 0.5) {
                servo.setPower(-1);
            }
            else {
                servo.setPower(0);
            }
        }
    }
}
