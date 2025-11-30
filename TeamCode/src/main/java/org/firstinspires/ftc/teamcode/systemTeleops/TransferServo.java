package org.firstinspires.ftc.teamcode.systemTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "transfer servo")
@Config
public class TransferServo extends LinearOpMode {

    // Dashboard-adjustable values
    public static double UP_POS = 0.55;
    public static double DOWN_POS = 0.2;

    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "myServo");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                servo.setPosition(DOWN_POS);
            }

            if (gamepad1.right_bumper) {
                servo.setPosition(UP_POS);
            }

            telemetry.addData("Current Position", servo.getPosition());
            telemetry.addData("UP_POS", UP_POS);
            telemetry.addData("DOWN_POS", DOWN_POS);
            telemetry.update();
        }
    }
}
