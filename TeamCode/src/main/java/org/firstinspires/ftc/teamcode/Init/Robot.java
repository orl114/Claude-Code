package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    // El shooter y la torreta NO están aquí: los controla DriveIntakeSlideRPinpoint directamente.

    public DcMotorEx mI1; // motor de ADELANTE del intake
    public DcMotorEx mI2; // motor de ATRÁS del intake

    private static Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        Robot.telemetry = telemetry;

        mI1 = hardwareMap.get(DcMotorEx.class, "mI1");
        mI1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mI1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mI2 = hardwareMap.get(DcMotorEx.class, "mI2");
        mI2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mI2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Robot listo");
        telemetry.update();
    }

    // Mueve los dos motores del intake juntos, para OpModes que no necesitan controlarlos
    // por separado (por ejemplo LimelightAutoAlign). El TeleOp principal los controla por
    // separado directamente con robot.mI1 / robot.mI2.
    public void moveIntake(double power) {
        mI1.setPower(power);
        mI2.setPower(power);
    }
}
