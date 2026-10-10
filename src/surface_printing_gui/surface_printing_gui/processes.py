"""Async process ownership and logs; workers never access Tk or invoke a shell."""
from dataclasses import dataclass
import os
import queue
import signal
import subprocess
import threading


@dataclass
class Job:
    name: str
    process: subprocess.Popen
    session: object
    kind: str


class ProcessManager:
    def __init__(self):
        self.events = queue.Queue()
        self.jobs = {}

    def start(self, name, argv, session, kind='command'):
        if name in self.jobs:
            raise ValueError(f'{name} is already running')
        process = subprocess.Popen(
            argv, cwd=session.workspace, env=session.environment(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            errors='replace', bufsize=1, start_new_session=True,
        )
        job = Job(name, process, session, kind)
        self.jobs[name] = job
        threading.Thread(target=self._read, args=(job,), daemon=True).start()
        return job

    def _read(self, job):
        try:
            for line in job.process.stdout:
                self.events.put(('log', job.name, line.rstrip()))
        finally:
            job.process.stdout.close()
            self.events.put(('exit', job.name, job.process.wait()))

    def reap(self, name):
        return self.jobs.pop(name, None)

    def interrupt(self, name):
        job = self.jobs.get(name)
        if job and job.process.poll() is None:
            try:
                os.killpg(job.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def moving(self):
        return [j for j in self.jobs.values() if j.kind == 'motion']
