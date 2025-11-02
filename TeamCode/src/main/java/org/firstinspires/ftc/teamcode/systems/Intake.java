package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    private LinearOpMode opMode;
    public DcMotor intakeMotor;
    public DcMotor transfer;

    // Constructor: initialize the motor
    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        transfer = opMode.hardwareMap.get(DcMotor.class, "transfer");
    }

    // Activate intake with a given power
    public void activateIntake(double power) {
        intakeMotor.setPower(power);
        transfer.setPower(power);
    }

    // Stop intake
}
