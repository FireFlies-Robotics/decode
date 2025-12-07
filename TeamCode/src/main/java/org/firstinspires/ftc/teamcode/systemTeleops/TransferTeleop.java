package org.firstinspires.ftc.teamcode.systemTeleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.systems.Transfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@TeleOp(name = "Transfare Control", group = "TeleOp")
@Config
public class TransferTeleop extends LinearOpMode {

    private Transfer intake;
    FtcDashboard dashboard;
    public static double up = 0.2;

    public static double down = 0.75;

    @Override
    public void runOpMode() {
        // Initialize the intake system
        intake = new Transfer(this);


        telemetry.addLine("Initialized — Ready to start");
        telemetry.update();

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {
            // If square (X) button is pressed, run intake
            if (gamepad2.triangle){
                intake.setTransferPower(up);
            }
            if (gamepad2.cross){
                intake.setTransferPower(down);
            }
            telemetry.addData("Intake Power", intake.intakeMotor.getPower());
            telemetry.update();
        }
    }
}
