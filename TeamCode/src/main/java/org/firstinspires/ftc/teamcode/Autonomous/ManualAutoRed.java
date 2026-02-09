package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Adyn Red", preselectTeleOp = "TeleOp")

public class ManualAutoRed extends ManualAuto {
    @Override
    public Paths getPaths() {
        return new PathsRed(pedro);
    }
}
