"""Policy-tick-only observation delay; never used in the motor control path."""
from collections import deque
import numpy as np


class PolicyQdotDelay:
    def __init__(self, steps):
        if not isinstance(steps, int) or steps < 0:
            raise ValueError('qdot delay steps must be a nonnegative integer')
        self.steps = steps
        self.history = deque(maxlen=max(1, steps))

    def sample(self, current, feedback_rx):
        current = np.asarray(current, dtype=np.float32).copy()
        stamps = np.asarray(feedback_rx, dtype=float).copy()
        if not np.all(np.isfinite(current)) or not np.all(np.isfinite(stamps)):
            raise ValueError('nonfinite qdot or source timestamp')
        if not self.history:
            self.history.extend((current.copy(), stamps.copy()) for _ in range(self.steps))
        result = (current, stamps) if self.steps == 0 else self.history[0]
        self.history.append((current, stamps))
        return result[0].copy(), result[1].copy()
