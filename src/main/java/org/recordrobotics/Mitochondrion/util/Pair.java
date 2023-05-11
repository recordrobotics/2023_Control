package org.recordrobotics.Mitocondrion.util;

public class Pair<T, K> {

	private T _key;
	private K _value;

	public Pair(T t, K k) {
		_key = t;
		_value = k;
	}

	public T getKey() {
		return _key;
	}

	public K getValue() {
		return _value;
	}

}
