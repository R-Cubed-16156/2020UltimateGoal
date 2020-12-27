package org.firstinspires.ftc.teamcode.breakout;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_F;
import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_R;

/**
 * Class used for all information needed for the robot.
 */
public class Robot {

    /**
     * Enum for each motor on the robot.
     */
    public enum Motor {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
    }

    /**
     * Enum for the claw's positions, open and closed.
     */
    public enum ClawPos {
        OPEN(0.83d), CLOSED(1.0d);

        private double pos;

        ClawPos(double pos) { this.pos = pos; }
    }

    /**
     * Enum for the pull tabs' positions, open and closed for each side.
     */
    public enum PushBarPos {
        OUT(-0.5d), IN(0.5d);

        private double pos;

       PushBarPos(double pos) { this.pos = pos; }
    }

    //Motors
    private BreakoutMotor frontLeft = new BreakoutMotor();
    private BreakoutMotor frontRight = new BreakoutMotor();
    private BreakoutMotor backLeft = new BreakoutMotor();
    private BreakoutMotor backRight = new BreakoutMotor();
    private BreakoutMotor wheelIntake = new BreakoutMotor();
    private BreakoutMotor flyWheel = new BreakoutMotor();

    //Servos
    private BreakoutServo pushBar = new BreakoutServo();
    private BreakoutServo claw = new BreakoutServo();

    //Gyro
    private BreakoutREVGyro gyro = new BreakoutREVGyro();

    //Misc
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    /* Constructor */
    public Robot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Sets the power for each motor on the wheel intake.
     *
     * @param power Float from -1 to 1 indicating how fast the motor moves.
     */
    public void setWheelIntake(float power) {
        wheelIntake.setPower(-power);
    }

    /**
     * Sets the tab position based on the boolean input.
     *
     * @param open Boolean variable to open/close the tabs.
     */
    public void setPushBar(boolean open) {
        if (open) {
            pushBar.setPosition(PushBarPos.IN.pos);
        } else {
            pushBar.setPosition(PushBarPos.OUT.pos);
        }
    }

    /**
     * Sets claw position to open or closed based on the boolean input.
     *
     * @param open Boolean variable to open/close the claw.
     */
    public void setClaw(boolean open) {
        if (open) {
            claw.setPosition(ClawPos.OPEN.pos);
        } else {
            claw.setPosition(ClawPos.CLOSED.pos);
        }
    }

    /**
     * Gets the position of the claw.
     *
     * @return Returns claw position.
     */
    public double getClawPos() {
        return claw.getPosition();
    }

    /**
     * Sets the power of the arm motor.
     *
     * @param power Float from -1 to 1 to set how fast the motor moves.
     */


    /**
     * Same effect as setArmPower but also uses encoders to help keep it more accurate.
     *
     * @param power Power of the motor from -1 to 1.
     */


    /**
     * Method to set the power of the {@link Motor} you pass in to the given power float.
     *
     * @param motor {@link Motor} to set the power of.
     * @param power Power to set from -1 to 1.
     */
    public void setPower(Motor motor, float power) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setPower(power);
                break;
            case FRONT_RIGHT:
                frontRight.setPower(power);
                break;
            case BACK_LEFT:
                backLeft.setPower(power);
                break;
            case BACK_RIGHT:
                backRight.setPower(power);
                break;
        }
    }

    /**
     * Method to get the power of the motors.
     *
     * @param motor {@link Motor} to get the power of.
     * @return Float from -1 to 1 that the power of the motor is.
     */
    public double getPower(Motor motor) {
        switch (motor) {
            case FRONT_LEFT:
                return frontLeft.getPower();
            case FRONT_RIGHT:
                return frontRight.getPower();
            case BACK_LEFT:
                return backLeft.getPower();
            case BACK_RIGHT:
                return backRight.getPower();
            default:
                return 0;
        }
    }

    /**
     * Sets the run mode of the given {@link Motor}.
     *
     * @param motor {@link Motor} to set the run mode of.
     * @param mode {@link DcMotor.RunMode} to set the given motor to.
     */
    public void setRunMode(Motor motor, DcMotor.RunMode mode) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setMotorMode(mode);
                break;
            case FRONT_RIGHT:
                frontRight.setMotorMode(mode);
                break;
            case BACK_LEFT:
                backLeft.setMotorMode(mode);
                break;
            case BACK_RIGHT:
                backRight.setMotorMode(mode);
                break;
        }
    }

    /**
     * Sets target position for the motor's encoders.
     *
     * @param motor {@link Motor} to set the target position of.
     * @param pos Integer position to set the target of the encoder to.
     */
    public void setTargetPosition(Motor motor, int pos) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setTargetPosition(pos);
                break;
            case FRONT_RIGHT:
                frontRight.setTargetPosition(pos);
                break;
            case BACK_LEFT:
                backLeft.setTargetPosition(pos);
                break;
            case BACK_RIGHT:
                backRight.setTargetPosition(pos);
                break;
        }
    }

    /**
     * Checks if the given motor is busy.
     *
     * @param motor {@link Motor} to check.
     * @return True if the motor is busy, false if not busy.
     */
    public boolean isBusy(Motor motor) {
        switch (motor) {
            case FRONT_LEFT:
                return frontLeft.isBusy();
            case FRONT_RIGHT:
                return frontRight.isBusy();
            case BACK_LEFT:
                return backLeft.isBusy();
            case BACK_RIGHT:
                return backRight.isBusy();
            default:
                return false;
        }
    }

    /**
     * Gets the current position of the encoders.
     *
     * @param motor {@link Motor} to get position of.
     * @return Returns integer of the encoder's position.
     */
    public int getCurrentPosition(Motor motor) {
        switch (motor) {
            case FRONT_LEFT:
                return frontLeft.getCurrentPosition();
            case FRONT_RIGHT:
                return frontRight.getCurrentPosition();
            case BACK_LEFT:
                return backLeft.getCurrentPosition();
            case BACK_RIGHT:
                return backRight.getCurrentPosition();
            default:
                return 8008135;
        }
    }

    /**
     * Gets the angle of the gyro.
     *
     * @return Float of gyro z axis.
     */
    public float getAngle() {
        return gyro.getOrient(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    public void start() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        // Gyro init
        gyro.set(hardwareMap.get(gyro.IMU, "imu 1"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        telemetry.update();
        gyro.calibrate();
        telemetry.clearAll();
        telemetry.update();

        // Define and Initialize Motors
        frontLeft.set(hardwareMap.dcMotor.get("leftFront"));
        frontRight.set(hardwareMap.dcMotor.get("rightFront"));
        backLeft.set(hardwareMap.dcMotor.get("leftRear"));
        backRight.set(hardwareMap.dcMotor.get("rightRear"));

        wheelIntake.set(hardwareMap.dcMotor.get("wheelIntake"));
        flyWheel.set(hardwareMap.dcMotor.get("flyWheel"));

        pushBar.set(hardwareMap.servo.get("pushBar"));

        claw.set(hardwareMap.servo.get("claw"));

        //Set directions for motors
        //F = Clockwise while looking at axle
        //R = Counter clockwise while looking at axle
        frontLeft.setDirection(MOTOR_F);
        frontRight.setDirection(MOTOR_F);
        backLeft.setDirection(MOTOR_F);
        backRight.setDirection(MOTOR_F);

        wheelIntake.setDirection(MOTOR_F);
        flyWheel.setDirection(MOTOR_F);


        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        wheelIntake.setPower(0);
        flyWheel.setPower(0);

        pushBar.setPosition(PushBarPos.OUT.pos);

        claw.setPosition(ClawPos.OPEN.pos);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheelIntake.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
