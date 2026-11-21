package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    // El intake (mI1/mI2) y el shooter NO están aquí: los controla
    // DriveIntakeSlideRPinpoint directamente, con sus propios motores.

    public Servo servoEmpuje1;
    public Servo servoEmpuje2;

    private static Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        Robot.telemetry = telemetry;

        // =========================
        // SERVOS
        // =========================

        servoEmpuje1 = hardwareMap.get(
                Servo.class,
                "servoEmpuje1"
        );

        servoEmpuje2 = hardwareMap.get(
                Servo.class,
                "servoEmpuje2"
        );

        // Invertir servo 2
        servoEmpuje2.setDirection(
                Servo.Direction.REVERSE
        );

        telemetry.addLine("Robot listo");
        telemetry.update();
    }

    // =========================
    // SERVOS EMPUJE
    // =========================

    public void moveServos() {
        servoEmpuje1.setPosition(1);
        servoEmpuje2.setPosition(1);
    }

    public void stopServos() {
        servoEmpuje1.setPosition(0.5);
        servoEmpuje2.setPosition(0.5);
    }
}
