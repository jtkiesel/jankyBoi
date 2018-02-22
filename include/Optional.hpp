#ifndef OPTIONAL_HPP_
#define OPTIONAL_HPP_

#include <memory>

namespace bns {

/**
 * A container object which may or may not contain a non-null value.
 * If a value is present, <code>isPresent()</code> will return <code>true</code>
 * and <code>get()</code> will return the value.
 *
 * Additional methods that depend on the presence or absence of a contained
 * value are provided, such as <code>orElse()</code> (return a default value if
 * value not present) and <code>ifPresent()</code> (execute a block of code if
 * the value is present).
 *
 * This is a <a href="../lang/doc-files/ValueBased.html">value-based</a> class;
 * use of identity-sensitive operations (including reference equality
 * (<code>==</code>), identity hash code, or synchronization) on instances of
 * <code>Optional</code> may have unpredictable results and should be avoided.
 */
template <typename T>
class Optional {
public:
	/**
	 * Returns an empty <code>Optional</code> instance. No value is present for
	 * this Optional.
	 *
	 * @param <T>  Type of the non-existent value.
	 * @return     An empty <code>Optional</code>.
	 */
	static Optional<T> empty() {
		return Optional();
	}
	Optional(const Optional<T> &other) : Optional(other.get()) {}
	/**
	 * Returns an <code>Optional</code> with the specified present value.
	 *
	 * @param <T>    Type of the value.
	 * @param value  The value to be present.
	 * @return       An <code>Optional</code> with the value present.
	 */
	static Optional<T> of(T value) {
		return Optional(value);
	}
	/**
	 * Returns the value.
	 *
	 * @return  The value held by this <code>Optional</code>.
	 */
	T get() const {
		return *value.get();
	}
	/**
	 * Return <code>true</code> if there is a value present, otherwise
	 * <code>false</code>.
	 *
	 * @return  <code>true</code> if there is a value present, otherwise
	 * <code>false</code>.
	 */
	bool isPresent() const {
		if (value) {
			return true;
		}
		return false;
	}
	/**
	 * Return the value if present, otherwise return <code>other</code>.
	 *
	 * @param other  The value to be returned if there is no value present.
	 * @return       The value, if present, otherwise <code>other</code>.
	 */
	T orElse(T other) const {
		return isPresent() ? value : other;
	}
private:
	/**
	 * A pointer to the value.
	 */
	const std::unique_ptr<T> value;
	/**
	 * Constructs an empty instance.
	 */
	Optional() {}
	/**
	 * Constructs an instance with the value present.
	 *
	 * @param value  The value to be present.
	 */
	Optional(T value) : value(std::unique_ptr<T>(new T(value))) {}
};

}  // namespace bns

#endif  // OPTIONAL_HPP_
