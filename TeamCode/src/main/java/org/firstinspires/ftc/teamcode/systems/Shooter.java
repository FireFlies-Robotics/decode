package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooter {
    private LinearOpMode opMode;
    public CRServo tunet;
    public CRServo houd;
    public DcMotor shotingMotor;

    public Shooter (LinearOpMode opMode) {
        this.opMode = opMode;
   //     tunet = opMode.hardwareMap.get(CRServo.class, "tunet");
        shotingMotor = opMode.hardwareMap.get(DcMotor.class, "shotingMotor");
     //   feywhed = opMode.hardwareMap.get(DcMotor.class, "feywhed");
    }
//    public void
    public void setShotingPower(double power) {
        shotingMotor.setPower(power);
    }
}
