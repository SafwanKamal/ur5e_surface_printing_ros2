"""Dedicated mock rehearsal view; commands reuse the main app's plan gate."""
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk

from .commands import ros_run
from .mock_results import MockFeedback, path_length_mm, read_path


class MockWindow:
    def __init__(self, app):
        self.app = app
        self.session = app.session()
        self.feedback = MockFeedback()
        self.points = []
        self.path_key = None
        self.started = None
        self.cancelled = False
        self.observed = False
        self.job_key = None
        self.duration_key = None
        self.cached_duration = None
        self.window = tk.Toplevel(app.root)
        self.window.title('UR5e | Mock motion verification | Domain 78')
        self.window.geometry('1160x900')
        self.window.minsize(960, 720)
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        ttk.Label(self.window, text='MOCK HARDWARE  |  DOMAIN 78  |  EXTRUSION OFF',
                  style='Heading.TLabel', padding=14).pack(anchor='w')
        ttk.Label(self.window, text='Blue: requested CSV path. Orange: live mock TCP during rehearsal. '
                  'Use RViz for the full robot, orientation and collision scene.', padding=(14, 0)).pack(anchor='w')
        self.message = tk.StringVar(value='Connect live state, capture a path, then plan and check.')
        ttk.Label(self.window, textvariable=self.message, wraplength=1100, padding=12).pack(anchor='w')
        columns = ttk.Frame(self.window, padding=(14, 0))
        columns.pack(fill='both', expand=True)
        controls = ttk.Frame(columns)
        controls.pack(side='left', fill='y', padx=(0, 15))
        plots = ttk.Frame(columns)
        plots.pack(side='left', fill='both', expand=True)
        self.buttons = []
        ttk.Checkbutton(controls, text='Physical drivers stopped; pump power off',
                        variable=app.mock_isolated).pack(anchor='w', pady=6)
        for label, fn in [('Launch mock robot + RViz', app.launch_stack),
                          ('Connect live mock state', self.connect)]:
            self.button(controls, label, fn)
        form = ttk.Frame(controls)
        form.pack(fill='x')
        for key, label in [('shape', 'Shape'), ('size', 'Size / diameter (mm)'),
                           ('height', 'Tube height (mm)'), ('pitch', 'Tube pitch (mm)'),
                           ('tube_step', 'Tube spacing (mm)'), ('line', 'Line ID'),
                           ('speed', 'TCP speed (mm/s)'), ('duration', 'Duration budget (s)')]:
            row = form.grid_size()[1]
            ttk.Label(form, text=label).grid(row=row, column=0, sticky='w', pady=3)
            if key == 'shape':
                widget = ttk.Combobox(form, textvariable=app.variables[key],
                                      values=('line', 'circle', 'square', 'tube'), state='readonly', width=12)
            else:
                widget = ttk.Entry(form, textvariable=app.variables[key], width=14)
            widget.grid(row=row, column=1, padx=8, pady=3)
        ttk.Label(controls, text='Other path/plan settings and file locations\nare shared with the main Path / Plan tabs.',
                  style='Note.TLabel').pack(anchor='w', pady=6)
        self.button(controls, '1. Capture at current mock tip', app.capture_path)
        self.button(controls, '2. Plan + RViz preview', app.plan_path)
        self.button(controls, '3. Check saved plan', app.check_plan)
        ttk.Checkbutton(controls, text='I reviewed the full path and RViz scene',
                        variable=app.confirmed).pack(anchor='w', pady=8)
        self.execute = self.button(controls, '4. Rehearse on mock hardware', self.rehearse)
        ttk.Button(controls, text='REQUEST MOCK ROBOT STOP', command=self.stop).pack(fill='x', pady=6)
        self.summary = tk.StringVar(value='No path loaded')
        ttk.Label(plots, textvariable=self.summary, wraplength=700).pack(anchor='w', pady=6)
        self.canvas = tk.Canvas(plots, bg='#0c1726', highlightthickness=0, height=340)
        self.canvas.pack(fill='both', expand=True)
        self.live = tk.StringVar(value='No live feedback')
        ttk.Label(plots, textvariable=self.live, wraplength=700).pack(anchor='w', pady=8)
        self.joints = ttk.Treeview(plots, columns=('angle',), show='tree headings', height=6)
        self.joints.heading('#0', text='Live mock joint')
        self.joints.heading('angle', text='Degrees')
        self.joints.column('#0', width=260)
        self.joints.column('angle', width=120)
        self.joints.pack(fill='x')
        self.progress = ttk.Progressbar(self.window, mode='determinate', maximum=100)
        self.progress.pack(fill='x', padx=14, pady=8)
        self.progress_text = tk.StringVar(value='No rehearsal run | progress is elapsed / planned time, not completion proof')
        ttk.Label(self.window, textvariable=self.progress_text, padding=(14, 0)).pack(anchor='w')
        self.logs = tk.Text(self.window, height=6, bg='#0c1726', fg='#d5e3ee', wrap='word', state='disabled')
        self.logs.pack(fill='x', padx=14, pady=10)
        self.tick_after = self.window.after(100, self.tick)

    def button(self, parent, label, fn):
        button = ttk.Button(parent, text=label, command=lambda: self.app.guard(lambda: self.action(fn)))
        button.pack(fill='x', pady=4)
        self.buttons.append(button)
        return button

    def require_mock(self):
        if self.app.session() != self.session or self.session.mode != 'mock' or self.session.domain != 78:
            raise ValueError('This window is pinned to mock domain 78. Restore its Session settings or close it.')
        if not self.app.mock_isolated.get():
            raise ValueError('Confirm mock isolation in the Session tab: physical drivers stopped and pump power disconnected.')

    def same_session(self):
        try:
            return self.app.session() == self.session
        except (ValueError, OSError):
            return False

    def action(self, fn):
        self.require_mock()
        fn()

    def connect(self):
        if 'Mock telemetry' in self.app.processes.jobs:
            return
        self.app.run('Mock telemetry', ros_run('surface_printing_gui', 'mock_monitor'), background=True)

    def rehearse(self):
        if not self.feedback.fresh():
            raise ValueError('Connect live mock state and wait for fresh joint/TF feedback before rehearsing here.')
        self.app.extrude.set(False)
        self.app.execute_plan()

    def stop(self):
        self.cancelled = True
        self.app.request_stop(session=self.session)

    def started_job(self, name):
        self.append(name, 'Started')
        if name in ('Capture path', 'Plan path'):
            self.observed = False
            self.job_key = None
            self.feedback.trace = []
            self.feedback.samples = 0
        self.message.set(name + ' running | mock domain 78')
        if name == 'Execute plan':
            self.feedback.start()
            self.started = time.monotonic()
            self.cancelled = False
            self.job_key = self.app.job_key()
            self.observed = False
            self.message.set('REHEARSING | motion is running on mock hardware; no pump output')

    def finished_job(self, name, code):
        self.append(name, f'Exited with code {code}')
        if name == 'Execute plan':
            self.feedback.recording = False
            self.observed = (code == 0 and not self.cancelled and self.feedback.samples > 0 and
                             self.feedback.fresh() and self.job_key == self.app.job_key())
            if self.cancelled:
                result = 'STOP REQUESTED | rehearsal is not verified'
            elif code:
                result = 'REHEARSAL FAILED | inspect output; fix and plan/check again'
            elif self.observed:
                result = 'MOCK REHEARSAL COMPLETED | executor succeeded with live TCP/joint feedback'
            else:
                result = 'Executor completed, but fresh motion feedback was unavailable | result is unverified'
            self.message.set(result + ' | Physical execution still needs its own fresh plan/check.')
            self.started = None
        elif name == 'Mock telemetry':
            self.message.set('Live monitor stopped | reconnect to observe mock motion')
        elif code:
            self.message.set(name + ' failed | inspect the output below')
        else:
            self.message.set(name + ' completed | ' + self.app.plan_state.get())

    def append(self, name, text):
        self.logs.configure(state='normal')
        self.logs.insert('end', f'[{name}] {text}\n')
        lines = int(self.logs.index('end-1c').split('.')[0])
        if lines > 400:
            self.logs.delete('1.0', f'{lines-400}.0')
        self.logs.see('end')
        self.logs.configure(state='disabled')

    def update_path(self):
        if self.feedback.recording:
            return
        filename = Path(self.app.get('world_csv')).expanduser()
        try:
            stat = filename.stat()
            key = (str(filename), stat.st_mtime_ns, stat.st_size, self.app.get('line'))
        except OSError:
            key = None
        if key == self.path_key:
            return
        self.path_key = key
        self.feedback.trace = []
        self.observed = False
        try:
            self.points = read_path(filename, self.app.get('line'))
            self.summary.set(f'{len(self.points)} requested points | {path_length_mm(self.points):.2f} mm TCP path | '
                             f'world meters | {filename}')
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.points = []
            self.summary.set('Path unavailable: ' + str(exc))

    def plan_duration(self):
        try:
            import yaml
            filename = Path(self.app.get('trajectory')).expanduser()
            stat = filename.stat()
            key = (str(filename), stat.st_mtime_ns, stat.st_size)
            if key == self.duration_key:
                return self.cached_duration
            self.duration_key = key
            self.cached_duration = None
            data = yaml.safe_load(filename.read_text())
            value = float(data['duration_sec'])
            self.cached_duration = value if math.isfinite(value) and value > 0 else None
            return self.cached_duration
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError):
            return None

    def tick(self):
        if not self.window.winfo_exists():
            return
        same = self.same_session()
        if not same:
            self.message.set('SESSION CHANGED | mock controls paused; this window cannot command physical hardware')
        busy = self.app.task is not None
        for button in self.buttons:
            button.configure(state='normal' if same and not busy else 'disabled')
        try:
            signature = self.app.signature()
        except OSError:
            signature = None
        ready = (same and not busy and self.app.mock_isolated.get() and self.app.confirmed.get() and self.feedback.fresh() and
                 self.app.gate.ready(signature))
        self.execute.configure(state='normal' if ready else 'disabled')
        self.update_path()
        fresh = self.feedback.fresh()
        tcp = self.feedback.data.get('tcp')
        self.live.set(('LIVE' if fresh else 'STALE / WAITING') + ' | ' +
                      ('TCP world (mm): ' + ', '.join(f'{v*1000:.2f}' for v in tcp) if tcp else 'No TCP received') +
                      f' | {self.feedback.samples} rehearsal samples')
        tilt = self.feedback.data.get('nozzle_tilt_deg')
        if isinstance(tilt, (int, float)) and math.isfinite(tilt):
            self.live.set(self.live.get() + f' | Nozzle {tilt:.1f} deg from down')
        for item in self.joints.get_children():
            self.joints.delete(item)
        for name, value in self.feedback.data.get('joints', {}).items():
            self.joints.insert('', 'end', text=name, values=(f'{math.degrees(value):.2f}',))
        duration = self.plan_duration()
        if self.started is not None:
            elapsed = time.monotonic() - self.started
            percent = min(99, 100 * elapsed / duration) if duration else 0
            self.progress['value'] = percent
            self.progress_text.set(f'Elapsed {elapsed:.1f}s | planned {duration:.1f}s | time estimate only'
                                   if duration else f'Elapsed {elapsed:.1f}s | planned duration unavailable')
        elif self.observed:
            self.progress['value'] = 100
            self.progress_text.set('Executor completed | live feedback observed | review the trace and RViz')
        else:
            self.progress['value'] = 0
            self.progress_text.set(f'Planned duration: {duration:.2f}s | ' + self.app.plan_state.get()
                                   if duration else self.app.plan_state.get())
        if self.started is None and self.job_key is not None and self.job_key != self.app.job_key():
            self.observed = False
            self.job_key = None
            self.message.set('JOB CHANGED | previous rehearsal applies only to the previous files/settings')
        self.draw(fresh)
        self.tick_after = self.window.after(250, self.tick)

    def draw(self, fresh):
        canvas = self.canvas
        canvas.delete('all')
        width, height = canvas.winfo_width(), canvas.winfo_height()
        all_points = self.points + self.feedback.trace
        tcp = self.feedback.data.get('tcp')
        if tcp:
            all_points = all_points + [tcp]
        if not all_points:
            canvas.create_text(width/2, height/2, text='Capture or load a world-meter path to see the projections', fill='#d5e3ee')
            return
        for index, (axis, title) in enumerate(((1, 'TOP | World X / Y'), (2, 'SIDE | World X / Z'))):
            left, top = index*width/2+35, 40
            right, bottom = (index+1)*width/2-20, height-30
            values = [(p[0]*1000, p[axis]*1000) for p in all_points]
            xmin, xmax = min(p[0] for p in values), max(p[0] for p in values)
            ymin, ymax = min(p[1] for p in values), max(p[1] for p in values)
            scale = max(0.001, min((right-left)/max(xmax-xmin, 1), (bottom-top)/max(ymax-ymin, 1)) * 0.85)
            cx, cy = (left+right)/2, (top+bottom)/2
            def project(point):
                return (cx+(point[0]*1000-(xmin+xmax)/2)*scale,
                        cy-(point[axis]*1000-(ymin+ymax)/2)*scale)
            canvas.create_text(left, 16, text=title, fill='#d5e3ee', anchor='w')
            canvas.create_text(left, height-12, text=f'X {xmin:.1f}..{xmax:.1f} mm | '
                               f'{"Y" if axis==1 else "Z"} {ymin:.1f}..{ymax:.1f} mm', fill='#94a3b8', anchor='w')
            for points, color in ((self.points, '#60a5fa'), (self.feedback.trace, '#fb923c')):
                if len(points) > 1:
                    stride = max(1, len(points)//1500)
                    selected = points[::stride]
                    if selected[-1] != points[-1]:
                        selected = selected + [points[-1]]
                    canvas.create_line(*[value for p in selected for value in project(p)], fill=color, width=2)
            if self.points:
                x, y = project(self.points[0])
                canvas.create_oval(x-4, y-4, x+4, y+4, fill='#60a5fa', outline='')
            if tcp:
                x, y = project(tcp)
                canvas.create_oval(x-5, y-5, x+5, y+5, fill='#34d399' if fresh else '#94a3b8', outline='white')

    def close(self):
        job = self.app.processes.jobs.get('Mock telemetry')
        if job and job.session == self.session:
            self.app.processes.interrupt('Mock telemetry')
        self.window.after_cancel(self.tick_after)
        self.app.mock_window = None
        self.window.destroy()
