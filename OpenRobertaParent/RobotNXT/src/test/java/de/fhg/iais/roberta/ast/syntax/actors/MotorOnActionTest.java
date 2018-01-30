package de.fhg.iais.roberta.ast.syntax.actors;

import org.junit.Test;

import de.fhg.iais.roberta.util.RobertaProperties;
import de.fhg.iais.roberta.util.Util1;
import de.fhg.iais.roberta.util.test.nxt.HelperNxtForTest;

public class MotorOnActionTest {
    HelperNxtForTest h = new HelperNxtForTest(new RobertaProperties(Util1.loadProperties(null)));

    @Test
    public void motorOn() throws Exception {
        String a = "OnFwdReg(OUT_B,SpeedTest(30),OUT_REGMODE_SPEED);OnFwdReg(OUT_C, SpeedTest(50), OUT_REGMODE_SPEED);";

        this.h.assertCodeIsOk(a, "/ast/actions/action_MotorOn.xml");
    }

    @Test
    public void motorOnFor() throws Exception {
        String a = "RotateMotor(OUT_B,SpeedTest(30), 360 * 1);";

        this.h.assertCodeIsOk(a, "/ast/actions/action_MotorOnFor.xml");
    }
}