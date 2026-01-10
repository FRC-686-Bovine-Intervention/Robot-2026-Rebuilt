package frc.util;

public class Container<T> {
	public T inner;
	private Container(T inner) {
		this.inner = inner;
	}
	public static <T> Container<T> of(T inner) {
		return new Container<>(inner);
	}
}
