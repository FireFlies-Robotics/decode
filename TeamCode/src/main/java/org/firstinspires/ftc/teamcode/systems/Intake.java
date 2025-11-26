package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private LinearOpMode opMode;
    public DcMotor intakeMotor;
    public DcMotor transfer;

//    public Servo transferServo;
    public CRServo transferServo;
//    public Servo transferServo;

    // Constructor: initialize the motor
    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        transfer = opMode.hardwareMap.get(DcMotor.class, "transfer");
        transferServo = opMode.hardwareMap.get(CRServo.class, "transferServo");
//        transferServo = opMode.hardwareMap.get(Servo.class, "transferServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

//        transferServo = opMode.hardwareMap.get(Servo.class, "transferServo");

    }

    // Activate intake with a given power
    public void activateIntake(double power) {
        intakeMotor.setPower(power);
        transfer.setPower(power);
    }
    public void setFinalTransferPower(double power){
        transferServo.setPower(power);
//        transferServo.setPosition(power);
    }
    public void setFinalTransferPower(){

    }


    // Stop intake
}
