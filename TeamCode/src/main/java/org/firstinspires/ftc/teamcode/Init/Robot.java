package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    // El intake (mI1/mI2) y el shooter NO están aquí: los controla
    // DriveIntakeSlideRPinpoint directamente, con sus propios motores.

    private static Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        Robot.telemetry = telemetry;

        telemetry.addLine("Robot listo");
        telemetry.update();
    }
}
