package frc.robot.Framework;

import java.util.List;

public class Parameter<T> {
    private T value;

    private List<ParameterHandle<T>> handles;

    public ParameterHandle<T> getHandle(String name) {
        ParameterHandle<T> handle = new ParameterHandle<T>(this);
        handles.add(handle);
        return handle;
    }
}
