package org.firstinspires.ftc.teamcode.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Chassis {

    public static final double SPEED = 0.4;
    private final DcMotor rightFront, rightRear, leftFront, leftRear;
    private static Telemetry telemetry;

    // OpMode dueño del chasis (puede ser null). Se usa para salir de los while de los
    // movimientos con encoder cuando se presiona STOP.
    private final LinearOpMode opMode;

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
    }

    // Recomendado: new Chassis(hardwareMap, telemetry, this) desde un LinearOpMode.
    public Chassis(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        Chassis.telemetry = telemetry;
        this.opMode = opMode;

        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightRear = hardwareMap.get(DcMotor.class, "BR");
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftRear = hardwareMap.get(DcMotor.class, "BL");

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(double forwardPower, double leftPower, double turnPower) {

        double leftY;
        double leftX;
        double rightX;

        if (forwardPower < 0.2 && forwardPower > -0.2) {
            leftY = 0;
        } else {
            leftY = forwardPower;
        }

        if (leftPower < 0.2 && leftPower > -0.2) {
            leftX = 0;
        } else {
            leftX = leftPower;
        }

        if (turnPower < 0.2 && turnPower > -0.2) {
            rightX = 0;
        } else {
            rightX = turnPower;
        }

        if (leftX != 0 && leftY != 0 && rightX != 0) {
            rightFront.setPower((leftY + leftX + rightX) / 3);
            rightRear.setPower((leftY - leftX + rightX) / 3);
            leftFront.setPower((-leftY + leftX + rightX) / 3);
            leftRear.setPower((-leftY - leftX + rightX) / 3);

        } else if ((leftX != 0 && leftY != 0 && rightX == 0)
                || (leftX != 0 && leftY == 0 && rightX != 0)
                || (leftX == 0 && leftY != 0 && rightX != 0)) {

            rightFront.setPower((leftY + leftX + rightX) / 2);
            rightRear.setPower((leftY - leftX + rightX) / 2);
            leftFront.setPower((-leftY + leftX + rightX) / 2);
            leftRear.setPower((-leftY - leftX + rightX) / 2);

        } else if ((leftX != 0 && leftY == 0 && rightX == 0)
                || (leftX == 0 && leftY != 0 && rightX == 0)
                || (leftX == 0 && leftY == 0 && rightX != 0)) {

            rightFront.setPower(leftY + leftX + rightX);
            rightRear.setPower(leftY - leftX + rightX);
            leftFront.setPower(-leftY + leftX + rightX);
            leftRear.setPower(-leftY - leftX + rightX);

        } else {
            rightFront.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            leftRear.setPower(0);
        }
    }

    public void move2(double forwardPower, double leftPower, double turnPower) {

        forwardPower = normalizeInput(-forwardPower);
        leftPower = normalizeInput(-leftPower);
        turnPower = normalizeInput(-turnPower);

        double powerFactor = (countNonZero(
                forwardPower,
                leftPower,
                turnPower
        ) == 3) ? 3 :
                (countNonZero(
                        forwardPower,
                        leftPower,
                        turnPower
                ) == 2) ? 2 : 1;

        rightFront.setPower(
                (forwardPower + leftPower + turnPower) / powerFactor
        );

        rightRear.setPower(
                (forwardPower - leftPower + turnPower) / powerFactor
        );

        leftFront.setPower(
                (-forwardPower + leftPower + turnPower) / powerFactor
        );

        leftRear.setPower(
                (-forwardPower - leftPower + turnPower) / powerFactor
        );
    }

    // Solo para control programático. No usar directamente con joysticks.
    public void moveRaw(double forwardPower, double leftPower, double turnPower) {

        forwardPower = -forwardPower;
        leftPower = -leftPower;
        turnPower = -turnPower;

        int nonZero = countNonZero(
                forwardPower,
                leftPower,
                turnPower
        );

        double powerFactor =
                (nonZero == 3) ? 3 :
                        (nonZero == 2) ? 2 : 1;

        rightFront.setPower(
                (forwardPower + leftPower + turnPower) / powerFactor
        );

        rightRear.setPower(
                (forwardPower - leftPower + turnPower) / powerFactor
        );

        leftFront.setPower(
                (-forwardPower + leftPower + turnPower) / powerFactor
        );

        leftRear.setPower(
                (-forwardPower - leftPower + turnPower) / powerFactor
        );
    }

    // true mientras el OpMode siga corriendo. Con el OpMode conocido usa opModeIsActive();
    // si se creó con el constructor viejo (sin OpMode), usa la interrupción del hilo,
    // que el SDK activa al presionar STOP en un LinearOpMode.
    private boolean isActive() {
        if (opMode != null) {
            return opMode.opModeIsActive();
        }
        return !Thread.currentThread().isInterrupted();
    }

    private double normalizeInput(double value) {
        return (Math.abs(value) < 0.2) ? 0 : value;
    }

    private int countNonZero(double... values) {
        int count = 0;

        for (double v : values) {
            if (v != 0) {
                count++;
            }
        }

        return count;
    }

    private int calculateTargetTicks(double distanceIn) {

        int ticksPerRevolution = 753;
        double wheelCircumference = (43.982 / 2.54);

        return (int) ((distanceIn / wheelCircumference) * ticksPerRevolution);
    }

    private void setMotorPower(double power) {
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        leftRear.setPower(power);
    }

    private void resetEncoderMode() {
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void logEncoderValues() {
        telemetry.addData(
                "Right Front Position",
                rightFront.getCurrentPosition()
        );

        telemetry.addData(
                "Right Rear Position",
                rightRear.getCurrentPosition()
        );

        telemetry.addData(
                "Left Front Position",
                leftFront.getCurrentPosition()
        );

        telemetry.addData(
                "Left Rear Position",
                leftRear.getCurrentPosition()
        );

        telemetry.update();
    }

    public void forwardEncoder(double distanceIn, double speed) {

        int targetTicks = calculateTargetTicks(distanceIn);

        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() - targetTicks
        );

        rightRear.setTargetPosition(
                rightRear.getCurrentPosition() - targetTicks
        );

        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + targetTicks
        );

        leftRear.setTargetPosition(
                leftRear.getCurrentPosition() + targetTicks
        );

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(speed);

        while (isActive() &&
                rightFront.isBusy() &&
                rightRear.isBusy() &&
                leftFront.isBusy() &&
                leftRear.isBusy()) {

            logEncoderValues();
        }

        setMotorPower(0);
        resetEncoderMode();
    }

    public void turnRightEncoder(double distanceIn, double speed) {

        int targetTicks = calculateTargetTicks(distanceIn);

        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() - targetTicks
        );

        rightRear.setTargetPosition(
                rightRear.getCurrentPosition() - targetTicks
        );

        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() - targetTicks
        );

        leftRear.setTargetPosition(
                leftRear.getCurrentPosition() - targetTicks
        );

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(speed);

        while (isActive() &&
                rightFront.isBusy() &&
                rightRear.isBusy() &&
                leftFront.isBusy() &&
                leftRear.isBusy()) {

            logEncoderValues();
        }

        setMotorPower(0);
        resetEncoderMode();
    }

    public void leftRunEncoder(double distanceIn, double speed) {

        int targetTicks = calculateTargetTicks(distanceIn);

        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() - targetTicks
        );

        rightRear.setTargetPosition(
                rightRear.getCurrentPosition() + targetTicks
        );

        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() - targetTicks
        );

        leftRear.setTargetPosition(
                leftRear.getCurrentPosition() + targetTicks
        );

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(speed);

        while (isActive() &&
                rightFront.isBusy() &&
                rightRear.isBusy() &&
                leftFront.isBusy() &&
                leftRear.isBusy()) {

            logEncoderValues();
        }

        setMotorPower(0);
        resetEncoderMode();
    }

    public void forward(double power) {
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        leftFront.setPower(power);
        leftRear.setPower(power);
    }

    public void turnRight(double power) {
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
    }

    public void leftRun(double power) {
        rightFront.setPower(-power);
        rightRear.setPower(power);
        leftFront.setPower(-power);
        leftRear.setPower(power);
    }

    public void stopChassis() {
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
    }
}
