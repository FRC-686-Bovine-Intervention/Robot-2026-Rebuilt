package frc.robot.subsystems.drive.odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;

public class OdometryThread extends Thread {
	private static OdometryThread instance;
	public static OdometryThread getInstance() {if (instance == null) {instance = new OdometryThread();} return instance;}

	public static final int MAX_BUFFER_SIZE = 20;

	private OdometryThread() {
		this.setName("OdometryThread");
		this.setDaemon(true);
	}

	private final Lock signalsLock = new ReentrantLock();
	public final Lock odometryLock = new ReentrantLock();
	private BaseStatusSignal[] baseStatusSignals = new BaseStatusSignal[0];
	private final List<Signal> signals = new ArrayList<>(9);
	private final List<DoubleBuffer> timestampBuffers = new ArrayList<>(1);

	public DoubleBuffer registerPhoenixDoubleSignal(BaseStatusSignal statusSignal, DoubleUnaryOperator converter) {
		var buffer = new DoubleBuffer(MAX_BUFFER_SIZE);
		this.signalsLock.lock();
		this.odometryLock.lock();
		try {
			var signal = new PhoenixDoubleSignal(statusSignal, converter, buffer);
			this.signals.add(signal);
			var newBaseStatusSignals = new BaseStatusSignal[this.baseStatusSignals.length + 1];
			System.arraycopy(this.baseStatusSignals, 0, newBaseStatusSignals, 0, this.baseStatusSignals.length);
			newBaseStatusSignals[this.baseStatusSignals.length] = statusSignal;
			this.baseStatusSignals = newBaseStatusSignals;
		} finally {
			this.signalsLock.unlock();
			this.odometryLock.unlock();
		}
		return buffer;
	}
	public <T> Buffer<T> registerPhoenixSignal(StatusSignal<T> statusSignal, IntFunction<T[]> arrayConstructor) {
		var buffer = new Buffer<T>(MAX_BUFFER_SIZE, arrayConstructor);
		this.signalsLock.lock();
		this.odometryLock.lock();
		try {
			var signal = new PhoenixSignal<>(statusSignal, buffer);
			this.signals.add(signal);
			var newBaseStatusSignals = new BaseStatusSignal[this.baseStatusSignals.length + 1];
			System.arraycopy(this.baseStatusSignals, 0, newBaseStatusSignals, 0, this.baseStatusSignals.length);
			newBaseStatusSignals[this.baseStatusSignals.length] = statusSignal;
			this.baseStatusSignals = newBaseStatusSignals;
		} finally {
			this.signalsLock.unlock();
			this.odometryLock.unlock();
		}
		return buffer;
	}
	public <T> Buffer<T> registerPhoenixComboSignal(Function<double[], T> generator, IntFunction<T[]> arrayConstructor, BaseStatusSignal... statusSignals) {
		var buffer = new Buffer<T>(MAX_BUFFER_SIZE, arrayConstructor);
		this.signalsLock.lock();
		this.odometryLock.lock();
		try {
			var signal = new PhoenixComboSignal<>(statusSignals, buffer, generator);
			this.signals.add(signal);
			var newBaseStatusSignals = new BaseStatusSignal[this.baseStatusSignals.length + statusSignals.length];
			System.arraycopy(this.baseStatusSignals, 0, newBaseStatusSignals, 0, this.baseStatusSignals.length);
			System.arraycopy(statusSignals, 0, newBaseStatusSignals, this.baseStatusSignals.length, statusSignals.length);
			this.baseStatusSignals = newBaseStatusSignals;
		} finally {
			this.signalsLock.unlock();
			this.odometryLock.unlock();
		}
		return buffer;
	}
	public <T> Buffer<T> registerGenericSignal(Supplier<T> supplier, IntFunction<T[]> arrayConstructor) {
		var buffer = new Buffer<T>(MAX_BUFFER_SIZE, arrayConstructor);
		this.signalsLock.lock();
		this.odometryLock.lock();
		try {
			this.signals.add(new GenericSignal<>(supplier, buffer));
		} finally {
			this.signalsLock.unlock();
			this.odometryLock.unlock();
		}
		return buffer;
	}
	public DoubleBuffer registerGenericDoubleSignal(DoubleSupplier supplier, DoubleUnaryOperator converter) {
		var buffer = new DoubleBuffer(MAX_BUFFER_SIZE);
		this.signalsLock.lock();
		this.odometryLock.lock();
		try {
			var signal = new GenericDoubleSignal(supplier, converter, buffer);
			this.signals.add(signal);
			var newBaseStatusSignals = new BaseStatusSignal[this.baseStatusSignals.length + 1];
			System.arraycopy(this.baseStatusSignals, 0, newBaseStatusSignals, 0, this.baseStatusSignals.length);
		} finally {
			this.signalsLock.unlock();
			this.odometryLock.unlock();
		}
		return buffer;
	}
	public DoubleBuffer generateTimestampBuffer() {
		var buffer = new DoubleBuffer(MAX_BUFFER_SIZE);
		this.odometryLock.lock();
		try {
			this.timestampBuffers.add(buffer);
		} finally {
			this.odometryLock.unlock();
		}
		return buffer;
	}

	private static interface Signal {
		public void poll();
	}
	private static record PhoenixDoubleSignal(BaseStatusSignal statusSignal, DoubleUnaryOperator converter, DoubleBuffer buffer) implements Signal {
		@Override
		public void poll() {
			this.buffer.offer(this.converter.applyAsDouble(this.statusSignal.getValueAsDouble()));
		}
	}
	private static record PhoenixSignal<T>(StatusSignal<T> statusSignal, Buffer<T> buffer) implements Signal {
		@Override
		public void poll() {
			this.buffer.offer(this.statusSignal.getValue());
		}
	}
	private static record PhoenixComboSignal<T>(BaseStatusSignal[] statusSignals, Buffer<T> buffer, Function<double[], T> generator) implements Signal {
		@Override
		public void poll() {
			var array = new double[this.statusSignals.length];
			for (int i = 0; i < array.length; i++) {
				array[i] = this.statusSignals[i].getValueAsDouble();
			}
			var value = this.generator.apply(array);
			this.buffer.offer(value);
		}
	}
	private static record GenericDoubleSignal(DoubleSupplier supplier, DoubleUnaryOperator converter, DoubleBuffer buffer) implements Signal {
		@Override
		public void poll() {
			this.buffer.offer(this.converter.applyAsDouble(this.supplier.getAsDouble()));
		}
	}
	private static record GenericSignal<T>(Supplier<T> supplier, Buffer<T> buffer) implements Signal {
		@Override
		public void poll() {
			this.buffer.offer(this.supplier.get());
		}
	}

	@Override
	public void start() {
		if (!this.timestampBuffers.isEmpty()) {
			super.start();
		}
	}

	@Override
	public void run() {
		while (true) {
			this.signalsLock.lock();
			try {
				Thread.sleep((long) (1000.0 / DriveConstants.odometryLoopFrequencyHz));
				if (this.baseStatusSignals.length > 0) {
					BaseStatusSignal.refreshAll(this.baseStatusSignals);
				}
			} catch (InterruptedException e) {
				e.printStackTrace();
			} finally {
				this.signalsLock.unlock();
			}

			this.odometryLock.lock();
			try {
				var timestamp = Timer.getFPGATimestamp();
				var totalLatency = 0.0;
				for (var signal : this.baseStatusSignals) {
					totalLatency += signal.getTimestamp().getLatency();
				}
				if (this.baseStatusSignals.length > 0) {
					timestamp -= totalLatency / this.baseStatusSignals.length;
				}
				for (var buffer : this.timestampBuffers) {
					buffer.offer(timestamp);
				}
				for (var signal : this.signals) {
					signal.poll();
				}
			} finally {
				this.odometryLock.unlock();
			}
		}
	}

	public static class DoubleBuffer {
		private final double[] buffer;
		private int tail = 0;

		public DoubleBuffer(int capacity) {
			this.buffer = new double[capacity];
		}

		public void offer(double value) {
			if (this.tail < this.buffer.length) {
				this.buffer[this.tail] = value;
				this.tail += 1;
			} else {
				System.arraycopy(this.buffer, 1, this.buffer, 0, this.buffer.length - 1);
				this.buffer[this.buffer.length - 1] = value;
			}
		}

		public double[] popAll() {
			var retArray = new double[this.tail];
			System.arraycopy(this.buffer, 0, retArray, 0, this.tail);
			this.tail = 0;
			return retArray;
		}
	}
	public static class Buffer<T> {
		private final T[] buffer;
		private final IntFunction<T[]> arrayConstructor;
		private int tail = 0;

		public Buffer(int capacity, IntFunction<T[]> arrayConstructor) {
			this.arrayConstructor = arrayConstructor;
			this.buffer = this.arrayConstructor.apply(capacity);
		}

		public void offer(T value) {
			if (this.tail < this.buffer.length) {
				this.buffer[this.tail] = value;
				this.tail += 1;
			} else {
				System.arraycopy(this.buffer, 1, this.buffer, 0, this.buffer.length - 1);
				this.buffer[this.buffer.length - 1] = value;
			}
		}

		public T[] popAll() {
			var retArray = this.arrayConstructor.apply(this.tail);
			System.arraycopy(this.buffer, 0, retArray, 0, this.tail);
			this.tail = 0;
			return retArray;
		}
	}
}
