package org.firstinspires.ftc.teamcode.systemTeleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp(name = "Turret Control1", group = "TeleOp")
public class TurretTeleop extends LinearOpMode {


    Turret turret;
    AnalogInput analogInput;
    double sensorVoltage;

    @Override
    public void runOpMode() {
        // Initialize the intake system
        turret = new Turret(this);


        telemetry.addLine("Initialized — Ready to start");
        telemetry.update();
        analogInput = hardwareMap.get(AnalogInput.class, "turretAnalog");
        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {

            sensorVoltage = analogInput.getVoltage();
            telemetry.addData( "analog input sensor", sensorVoltage);

            turret.moveTurret(gamepad2.right_stick_x);
                // If square (X) button is pressed, run intake

                // If square (X) butt  on is pressed, run intake


            telemetry.addData("Turret power", turret.rightTurret.getPower());
            turret.updateTurretServoRotation();
            telemetry.update();
        }
    }
}

