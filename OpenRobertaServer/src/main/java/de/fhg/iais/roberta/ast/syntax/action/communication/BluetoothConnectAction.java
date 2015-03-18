package de.fhg.iais.roberta.ast.syntax.action.communication;

import de.fhg.iais.roberta.ast.syntax.BlocklyBlockProperties;
import de.fhg.iais.roberta.ast.syntax.BlocklyComment;
import de.fhg.iais.roberta.ast.syntax.BlocklyConstants;
import de.fhg.iais.roberta.ast.syntax.Phrase;
import de.fhg.iais.roberta.ast.syntax.action.Action;
import de.fhg.iais.roberta.ast.syntax.action.MotorDriveStopAction;
import de.fhg.iais.roberta.ast.syntax.expr.Expr;
import de.fhg.iais.roberta.ast.transformer.AstJaxbTransformerHelper;
import de.fhg.iais.roberta.ast.visitor.AstVisitor;
import de.fhg.iais.roberta.blockly.generated.Block;

public class BluetoothConnectAction<V>  extends Action<V>{
    private final Expr<V> _address;

    private BluetoothConnectAction(Expr<V> address, BlocklyBlockProperties properties, BlocklyComment comment) {
        super(Phrase.Kind.BLUETOOTH_CONNECT_ACTION, properties, comment);
        _address = address;
        setReadOnly();
    }

    /**
     * Creates instance of {@link MotorDriveStopAction}. This instance is read only and can not be modified.
     *
     * @param properties of the block (see {@link BlocklyBlockProperties}),
     * @param comment added from the user,
     * @return read only object of class {@link MotorDriveStopAction}
     */
    public static <V> BluetoothConnectAction<V> make(Expr<V> address, BlocklyBlockProperties properties, BlocklyComment comment) {
        return new BluetoothConnectAction<V>(address, properties, comment);
    }

    @Override
    public String toString() {
        return "BluetoothConnectAction [" + get_address().toString() + "]";
    }

    @Override
    protected V accept(AstVisitor<V> visitor) {
        return visitor.visitBluetoothConnectAction(this);
    }

    @Override
    public Block astToBlock() {
        Block jaxbDestination = new Block();
        AstJaxbTransformerHelper.setBasicProperties(this, jaxbDestination);
        
        AstJaxbTransformerHelper.addValue(jaxbDestination, BlocklyConstants.ADDRESS, get_address());
        
        return jaxbDestination;
    }

    public Expr<V> get_address() {
        return _address;
    }
}
