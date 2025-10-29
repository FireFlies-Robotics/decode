package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PID {
    private LinearOpMode opMode;
    // Dashboard-tunable PID constants
    private static double KP;
    private static double KI;      // <-- new integral gain (start small!)
    private static double KD;
    // Optional tuning parameters
    private static double MAX_POWER;
    private static double INTEGRAL_LIMIT; // anti-windup limit
    private static double DEADZONE; // degrees


    private double lastError = 0;
    private double lastTime = 0;
    private double integralSum = 0;


    public PID(double KP, double KI, double KD, double MAX_POWER, double INTEGRAL_LIMIT, double DEADZONE, LinearOpMode opMode){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.MAX_POWER = MAX_POWER;
        this.INTEGRAL_LIMIT = INTEGRAL_LIMIT;
        this.DEADZONE = DEADZONE;
        this.opMode = opMode;

    }

    public double calculatePIDValue(double position, double target){
        double error = position - target;

        double currentTime = opMode.getRuntime();
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 1e-3;

        // Deadzone
        if (Math.abs(error) < DEADZONE) error = 0;

        // Integral term (with anti-windup)
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));

        // Derivative term
        double derivative = (error - lastError) / dt;

        // PID Control
        double power = KP * error + KI * integralSum + KD * derivative;

        // Clamp output
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        // Set servo power
        lastError = error;
        lastTime = currentTime;

        return power;

        // Update memory
    }
    public void printValues(){
        pid.calculatePIDValue(lastError, 0);
        opMode.telemetry.addData("Tag ID", detection.id);
        opMode.telemetry.addData("Bearing (deg)", "%.2f", error);
        opMode.telemetry.addData("Servo Power", "%.3f", power);
        opMode.telemetry.addData("Integral Sum", "%.3f", integralSum);
        opMode.telemetry.addData("Distance (m)", "%.2f", detection.ftcPose.range);

    }
}
