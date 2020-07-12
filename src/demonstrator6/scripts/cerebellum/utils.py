try:
   import queue
except ImportError:
   import Queue as queue
import copy

class TimeDelay:
    """
    Time delay implemented as with a queue.
    Parameters
    ----------
    `delay`: Number of delay time steps (int)

    `initial_value`: Initial value (object)
    """
    def __init__(self, delay=5, initial_value=0):
        self.delay = delay
        self.Q = queue.Queue(delay)
        for _ in range(0,delay):
            self.Q.put(copy.copy(initial_value))
        self.out = initial_value

    @property
    def output(self):
        return self.out

    def step(self, input_data=0):
        self.out = self.Q.get() # pop
        self.Q.put(copy.copy(input_data))

if __name__ == "__main__":
    import numpy as np
    # Test time delay
    T = 15
    delay = TimeDelay(T)

    for k in range(0,2*T):
        val = k*k
        delay.step(val)
        print("u({}): {}, u({}): {}".format(k, val, k-T, delay.output))

    # Test with data struct?
    print("With dict:")
    T = 5
    val = {'a': 5, 'b': 7}
    delay = TimeDelay(T, val)
    for k in range(0,2*T):
        val['b'] += 1
        delay.step(val)
        print("u({}): {}, u({}): {}".format(k, val, k-T, delay.output))

    # Test with list
    print("With list:")
    T = 5
    val = [1, 2, 3]
    delay = TimeDelay(T, val)
    for k in range(0,2*T):
        val[2] += 1
        delay.step(val)
        print("u({}): {}, u({}): {}".format(k, val, k-T, delay.output))

    # Test with np array
    print("With np array")
    T = 5
    val = np.array([1, 2, 3])
    delay = TimeDelay(T, val)
    for k in range(0,2*T):
        val[2] += 1
        delay.step(val)
        print("u({}): {}, u({}): {}".format(k, val, k-T, delay.output))
