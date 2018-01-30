package de.fhg.iais.roberta.ast.syntax.stmt;

import org.junit.Ignore;
import org.junit.Test;

import de.fhg.iais.roberta.util.RobertaProperties;
import de.fhg.iais.roberta.util.Util1;
import de.fhg.iais.roberta.util.test.nxt.HelperNxtForTest;

@Ignore
public class WaitStmtTest {
    HelperNxtForTest h = new HelperNxtForTest(new RobertaProperties(Util1.loadProperties(null)));

    @Test
    public void test1() throws Exception {
        String a = "publicvoidrun(){if(TRUE){while(true){if(hal.isPressed(BrickKey.ENTER)==true){break;}hal.waitFor(15);}}}";

        this.h.assertCodeIsOk(a, "/ast/control/wait_stmt2.xml");
    }
}