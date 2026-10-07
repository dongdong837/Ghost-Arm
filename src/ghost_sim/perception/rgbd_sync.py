"""Bounded exact-stamp RGB/depth/info sync that recovers after simulation resets."""
class RGBDSync:
    def __init__(self, capacity=30):
        self.capacity = capacity
        self.frames = {}
        self.last = {}
        self.resets = 0

    def push(self, kind, stamp, message):
        if kind in self.last and stamp < self.last[kind]:
            self.frames.clear()
            self.last.clear()
            self.resets += 1
        self.last[kind] = stamp
        self.frames.setdefault(stamp, {})[kind] = message
        while len(self.frames) > self.capacity:
            del self.frames[next(iter(self.frames))]

    def latest(self):
        ready = [stamp for stamp, frame in self.frames.items()
                 if all(kind in frame for kind in ('rgb', 'depth', 'info'))]
        if not ready:
            return None
        stamp = max(ready)
        frame = self.frames[stamp]
        for old in list(self.frames):
            if old <= stamp:
                del self.frames[old]
        return frame
