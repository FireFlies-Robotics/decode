package org.firstinspires.ftc.teamcode.systemTeleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp(name = "Turret Control", group = "TeleOp")
public class TurretTeleop extends LinearOpMode {

    Turret turret;

    @Override
    public void runOpMode() {
        // Initialize the intake system
        turret = new Turret(this);

        telemetry.addLine("Initialized — Ready to start");
        telemetry.update();

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {
            turret.moveTurret(gamepad2.right_stick_x);

                // If square (X) button is pressed, run intake


            telemetry.addData("Turret power", turret.rightTurret.getPower());
            telemetry.update();
        }
    }
}

