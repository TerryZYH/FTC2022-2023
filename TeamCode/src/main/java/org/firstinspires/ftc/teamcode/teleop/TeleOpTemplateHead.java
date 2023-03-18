package org.firstinspires.ftc.teamcode.teleop;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;



@TeleOp(name="TeleOpTemplateHead", group="Linear Opmode")

public class TeleOpTemplateHead extends LinearOpMode{
    private DcMotorEx liftMotor = null;
    private Servo rightClaw = null;
    private Servo leftClaw = null;
    private DcMotorEx yuntai = null;:


    @Override

    public void runOpMode()
    {
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        yuntai = hardwareMap.get(DcMotorEx.class, "yuntai");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yuntai.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yuntai.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive())
        {
            int target;//滑轨目标位置
            double speed;//滑轨功率

            // 抬升电机取值
            int pos_high = -1800;
            int pos_mid = -1300;
            int pos_low = -800;
            int pos_move = -300;
            int pos_rest = 0;

            // 云台电机取值
            int pos_left = -260;
            int pos_right = 260;
            int pos_front = 0;

            // TODO: 抬升控制

            // TODO: 云台控制

            // TODO: 爪子控制


            telemetry.addData("lift_Encoder_Val", liftMotor.getCurrentPosition());
            telemetry.addData("yuntai_Encoder_Val", yuntai.getCurrentPosition());

            telemetry.addData("Volocity_Val", liftMotor.getVelocity());
            telemetry.update();
        }
    }
}


