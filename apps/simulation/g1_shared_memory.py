"""Single-writer latest-frame mailbox, protected by a nonblocking file lock."""
import fcntl
import mmap
import os
from pathlib import Path


class SharedFeed:
    def __init__(self, kind, size, directory="/dev/shm"):
        self.path = Path(directory) / f"g1-{kind}-{os.getuid()}"
        self.owner = os.fdopen(os.open(str(self.path) + '.owner',
            os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600), 'r+b')
        self.file = self.memory = None
        try:
            fcntl.flock(self.owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.file = os.fdopen(os.open(self.path,
                os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600), 'r+b')
            fcntl.flock(self.file, fcntl.LOCK_EX)
            try:
                self.file.truncate(size)
                self.memory = mmap.mmap(self.file.fileno(), 0)
                self.memory[:] = bytes(size)
            finally:
                fcntl.flock(self.file, fcntl.LOCK_UN)
        except Exception:
            if self.memory is not None:
                self.memory.close()
            if self.file is not None:
                self.file.close()
            self.owner.close()
            raise

    def publish(self, packet):
        if len(packet) != len(self.memory):
            raise ValueError('Shared frame size mismatch')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        try:
            self.memory[:] = packet
        finally:
            fcntl.flock(self.file, fcntl.LOCK_UN)
        return True

    def close(self):
        self.memory.close()
        self.file.close()
        self.path.unlink(missing_ok=True)
        self.owner.close()
