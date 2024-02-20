package frc.robot.Framework;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

public class Parameter<T> {
    private T value;

    private List<ParameterHandle<T>> handles;
    private ParameterHandle<T> controllingHandle;
    //handles may request exclusive control, where control over the parameter will not be relinquished unless
    // the controlling handle explicitly releases it
    private boolean exclusiveControllingHandle; 

    public Consumer<T> onValueUpdated;

    public Parameter(T defaultValue) {
        value = defaultValue;
        handles = new LinkedList<ParameterHandle<T>>();
    }

    public ParameterHandle<T> getHandle(String name) {
        ParameterHandle<T> handle = new ParameterHandle<T>(this, name);
        handles.add(handle);
        return handle;
    }

    public void setValueUpdatedCallback(Consumer<T> callback) {
        this.onValueUpdated = callback;
    }

    public boolean takeControl(ParameterHandle<T> handle, boolean exclusive) {
        if(handle.equals(controllingHandle)) return true;
        if(exclusiveControllingHandle) return false;
        controllingHandle = handle;
        exclusiveControllingHandle = exclusive;
        return true;
    }

    public boolean releaseControl(ParameterHandle<T> handle) {
        if(!handle.equals(controllingHandle)) return false;
        exclusiveControllingHandle = false;
        controllingHandle = null;
        return true;
    }

    public boolean hasControl(ParameterHandle<T> handle) {
        return handle.equals(controllingHandle);
    }

    public boolean setValue(ParameterHandle<T> handle, T value) {
        if(!controllingHandle.equals(handle)) return false;
        this.value = value;
        if(this.onValueUpdated != null) this.onValueUpdated.accept(value);
        return true;
    }

    public T getValue() {
        return value;
    }
}
