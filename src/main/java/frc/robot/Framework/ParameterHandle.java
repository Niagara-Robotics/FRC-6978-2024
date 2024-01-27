package frc.robot.Framework;

public class ParameterHandle<T> {
    private Parameter<T> parent;
    private String name;



    public ParameterHandle(Parameter<T> parent, String name) {
        this.parent = parent;
        this.name = name;
    }

    public boolean set(T value) {
        return parent.setValue(this, value);
    }

    public T get() {
        return parent.getValue();
    }

    public boolean takeControl(boolean exclusive) {
        return parent.takeControl(this, exclusive);
    }

    public boolean hasControl() {
        return parent.hasControl(this);
    }

    public String getName() {
        return this.name;
    }
}
