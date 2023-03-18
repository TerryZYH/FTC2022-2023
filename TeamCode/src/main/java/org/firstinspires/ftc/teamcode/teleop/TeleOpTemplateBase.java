package org.firstinspires.ftc.teamcode.teleop;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOpTemplateBase", group="Linear Opmode")

public class TeleOpTemplateBase extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotorEx liftMotor = null;
    private Servo rightClaw = null;
    private Servo leftClaw = null;
    private DcMotorEx yuntai = null;



    @Override

    public void runOpMode()
    {
        // 配置四个底盘电机
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "rb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rf");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");

        // 对右侧电机旋转正方向进行反转
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // 四个底盘电机初始化为Break状态
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive())
        {
            // 读取手柄摇杆值
            double left_stick_x = 0;
            double left_stick_y = 0;
            double right_stick_x = 0;

            // 底盘运动控制
            double drive= 0;
            double turn= 0;
            double Hdrive=0;

            // 底盘电机功率设置
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);

            // telemetry 输出
            telemetry.addData("lift_stick_x", 0);
            telemetry.addData("lift_stick_y", 0);
            telemetry.addData("right_stick_x", 0);
            telemetry.update();
        }
    }
}


