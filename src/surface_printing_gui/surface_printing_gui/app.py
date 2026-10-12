"""Tk desktop workflow console. The existing ROS executors remain authoritative."""
import argparse
import hashlib
import json
from pathlib import Path
import queue
import re
import shlex
import shutil
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .commands import (Session, PlanGate, capture, convert, dispense, execution,
                       integer, number, path, plan, ros_run, stop_commands)
from .processes import ProcessManager


class PrintingApp:
    def __init__(self, root, workspace=None, pipeline=None):
        self.root = root
        self.root.title('UR5e | Surface Printing Console')
        self.root.geometry('1180x920')
        self.root.minsize(980, 740)
        self.processes = ProcessManager()
        self.mock_window = None
        self.gate = PlanGate()
        self.callbacks = {}
        self.task = None
        self.variables = {}
        self.sequence = 0
        self.last_mode = 'mock'
        self.confirmed = tk.BooleanVar(value=False)
        self.extrude = tk.BooleanVar(value=False)
        self.pump_confirmed = tk.BooleanVar(value=False)
        self.mock_isolated = tk.BooleanVar(value=False)
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('.', font=('DejaVu Sans', 10))
        style.configure('TNotebook.Tab', padding=(15, 8))
        style.configure('TButton', padding=(10, 7))
        style.configure('Heading.TLabel', font=('DejaVu Sans', 14, 'bold'))
        style.configure('Note.TLabel', foreground='#475569')
        header = tk.Frame(root, bg='#10283f', padx=22, pady=15)
        header.pack(fill='x')
        tk.Label(header, text='UR5e  /  SURFACE PRINTING', bg='#10283f', fg='white',
                 font=('DejaVu Sans', 18, 'bold')).pack(side='left')
        self.status = tk.StringVar(value='MOCK MODE | No workflow running')
        tk.Label(header, textvariable=self.status, bg='#10283f', fg='#9de0d9').pack(side='right')
        bar = ttk.Frame(root, padding=(15, 8))
        bar.pack(fill='x')
        ttk.Label(bar, text='Generate > register > plan > check in RViz > rehearse > print',
                  style='Note.TLabel').pack(side='left')
        ttk.Button(bar, text='REQUEST ROBOT + PUMP STOP', command=self.request_stop).pack(side='right')
        ttk.Button(bar, text='Open mock verification', command=lambda: self.guard(self.open_mock_window)).pack(side='right', padx=8)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill='both', expand=True, padx=14)
        self.pages = {name: self.page(name) for name in
                      ('1  Session', '2  Geometry', '3  Path', '4  Plan & Print', '5  Syringe', '6  Diagnostics')}
        self.setup_page(workspace or str(Path.home() / 'ur5e_ws'),
                        pipeline or str(Path.home() / 'Surface_printing_path_generation_pipeline'))
        self.geometry_page()
        self.path_page()
        self.print_page()
        self.pump_page()
        self.diagnostics_page()
        logbox = ttk.LabelFrame(root, text='Live output | commands run in the selected ROS domain', padding=8)
        logbox.pack(fill='both', padx=14, pady=10)
        self.logs = tk.Text(logbox, height=9, bg='#0c1726', fg='#d5e3ee',
                            insertbackground='white', font=('DejaVu Sans Mono', 9), wrap='word')
        self.logs.pack(fill='both', expand=True)
        self.logs.configure(state='disabled')
        self.plan_state = tk.StringVar(value='No checked plan | execute is locked')
        # The label is placed below the logs to remain visible from every tab.
        ttk.Label(root, textvariable=self.plan_state, padding=(15, 0, 15, 8)).pack(fill='x')
        for key in self.job_fields:
            self.variables[key].trace_add('write', self.invalidate)
        self.variables['mode'].trace_add('write', self.mode_changed)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.poll_after = self.root.after(100, self.poll)
        self.log('GUI', 'Source ROS Jazzy and the built workspace before launching. Mock mode is the default.')
        if not shutil.which('ros2'):
            self.log('GUI', 'ros2 is not on PATH. Geometry can still run; source ROS before using robot tools.')

    def page(self, name):
        container = ttk.Frame(self.notebook)
        self.notebook.add(container, text=name)
        canvas = tk.Canvas(container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        frame = ttk.Frame(canvas, padding=18)
        window = canvas.create_window((0, 0), window=frame, anchor='nw')
        frame.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda event: canvas.itemconfigure(window, width=event.width))
        frame.columnconfigure(1, weight=1)
        return frame

    def section(self, parent, text, note=''):
        row = parent.grid_size()[1]
        ttk.Label(parent, text=text, style='Heading.TLabel').grid(row=row, column=0, columnspan=3, sticky='w', pady=(14, 5))
        if note:
            ttk.Label(parent, text=note, style='Note.TLabel', wraplength=970).grid(
                row=row+1, column=0, columnspan=3, sticky='w', pady=(0, 8))

    def field(self, parent, key, label, default='', choices=None, browse=None):
        var = self.variables.setdefault(key, tk.StringVar(value=default))
        row = parent.grid_size()[1]
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky='w', padx=(0, 15), pady=4)
        widget = (ttk.Combobox(parent, textvariable=var, values=choices, state='readonly')
                  if choices else ttk.Entry(parent, textvariable=var))
        widget.grid(row=row, column=1, sticky='ew', pady=4)
        if browse:
            ttk.Button(parent, text='Browse...', command=lambda: self.browse(var, browse)).grid(row=row, column=2, padx=6)
        return var

    def buttons(self, parent, specs):
        frame = ttk.Frame(parent)
        frame.grid(row=parent.grid_size()[1], column=0, columnspan=3, sticky='w', pady=10)
        created=[]
        for label, callback in specs:
            button=ttk.Button(frame, text=label, command=lambda fn=callback: self.guard(fn))
            button.pack(side='left', padx=(0, 8))
            created.append(button)
        return created

    def checkbox(self, parent, label, var):
        ttk.Checkbutton(parent, text=label, variable=var).grid(
            row=parent.grid_size()[1], column=0, columnspan=3, sticky='w', pady=5)

    def browse(self, var, kind):
        initial = str(Path(var.get()).expanduser().parent)
        result = (filedialog.askdirectory(initialdir=initial) if kind == 'dir' else
                  filedialog.asksaveasfilename(initialdir=initial) if kind == 'save' else
                  filedialog.askopenfilename(initialdir=initial))
        if result:
            var.set(result)

    def get(self, key):
        return self.variables[key].get()

    def guard(self, action):
        try:
            action()
        except (ValueError, OSError, RuntimeError) as exc:
            messagebox.showerror('Cannot run this step', str(exc))

    def session(self):
        return Session(self.get('mode'), integer(self.get('domain'), 'ROS domain', 0, 232),
                       str(Path(self.get('workspace')).expanduser().resolve()))

    def open_mock_window(self):
        if self.mock_window is not None:
            self.mock_window.window.lift()
            return
        s = self.session()
        if s.mode != 'mock' or s.domain != 78:
            raise ValueError('Select mock mode and ROS domain 78 before opening mock verification.')
        from .mock_window import MockWindow
        self.mock_window = MockWindow(self)

    def mode_changed(self, *args):
        mode = self.get('mode')
        if mode != self.last_mode:
            self.variables['domain'].set('78' if mode == 'mock' else '77')
            self.last_mode = mode
            self.confirmed.set(False)
            self.extrude.set(False)
            self.pump_confirmed.set(False)
        self.invalidate()

    def invalidate(self, *args):
        self.gate.invalidate()
        self.update_execute_button()
        if hasattr(self, 'plan_state'):
            self.plan_state.set('Job changed | plan and check again before execution')

    def update_execute_button(self, *args):
        if hasattr(self, 'execute_button'):
            allowed=(self.gate.checked is not None and self.confirmed.get() and self.task is None
                     and not (self.get('mode')=='mock' and self.extrude.get()))
            self.execute_button.configure(state='normal' if allowed else 'disabled')

    def setup_page(self, workspace, pipeline):
        p = self.pages['1  Session']
        self.section(p, 'Connect the workspace', 'Run this console from a terminal that has sourced ROS Jazzy and this workspace. Child processes inherit that environment.')
        self.field(p, 'mode', 'Operating mode', 'mock', choices=('mock', 'physical'))
        self.field(p, 'domain', 'ROS domain', '78')
        self.field(p, 'workspace', 'ROS workspace root', workspace, browse='dir')
        self.field(p, 'pipeline', 'Offline geometry repository', pipeline, browse='dir')
        self.field(p, 'robot_ip', 'UR5e IP', '192.168.1.102')
        self.field(p, 'reverse_ip', 'ROS computer IP', '192.168.1.100')
        self.field(p, 'calibration', 'Robot-specific kinematics YAML', str(Path.home() / 'ur5e_calibration.yaml'), browse='file')
        self.field(p, 'pump_config', 'Syringe YAML (blank = package default)', '', browse='file')
        self.checkbox(p, 'For mock work: physical drivers are stopped and pump power is disconnected', self.mock_isolated)
        self.buttons(p, [('Launch robot + MoveIt + RViz', self.launch_stack),
                         ('Launch syringe controller', self.launch_syringe),
                         ('Physical preflight', lambda: self.run('Preflight', ros_run('surface_printing', 'hardware_preflight')))])
        self.section(p, 'Owned background processes', 'Select a process and interrupt it to shut it down. Stop requests target the domain used when each job was started.')
        self.job_list = tk.Listbox(p, height=5, exportselection=False)
        self.job_list.grid(row=p.grid_size()[1], column=0, columnspan=3, sticky='ew')
        self.buttons(p, [('Interrupt selected process', self.interrupt_selected)])

    def launch_stack(self):
        s = self.session()
        if s.simulation:
            if not self.mock_isolated.get():
                raise ValueError('Confirm physical drivers are stopped and pump power is disconnected')
            argv = ['ros2', 'launch', 'ur5e_probe_moveit_config', 'demo.launch.py']
        else:
            argv = ['ros2', 'launch', 'ur5e_probe_moveit_config', 'real_robot.launch.py',
                    'robot_ip:='+self.get('robot_ip'), 'reverse_ip:='+self.get('reverse_ip'),
                    'kinematics_params_file:='+path(self.get('calibration'), 'Robot calibration', True)]
        self.run('Robot stack', argv, background=True)

    def launch_syringe(self):
        if self.session().simulation:
            raise ValueError('Serial pump launch is disabled in mock mode')
        argv = ['ros2', 'launch', 'syringe_controller', 'syringe.launch.py']
        if self.get('pump_config').strip():
            argv.append('config:='+path(self.get('pump_config'), 'Syringe config', True))
        self.run('Syringe controller', argv, background=True)

    def geometry_page(self):
        p = self.pages['2  Geometry']
        self.section(p, 'Process a mesh offline', 'Use millimeter geometry for the raster export. Each step writes the output shown below; select it as the next input. No robot motion is requested.')
        self.field(p, 'mesh_input', 'Input mesh', '', browse='file')
        self.field(p, 'mesh_output', 'Output mesh', '/tmp/printing_mesh.ply', browse='save')
        self.field(p, 'normalization', 'Normalization JSON', '/tmp/printing_normalization.json', browse='save')
        self.field(p, 'source_up', 'Original up axis', 'z', choices=('x','-x','y','-y','z','-z'))
        self.buttons(p, [('Inspect', lambda: self.geometry('inspect')),
                         ('Prepare', lambda: self.geometry('prepare')),
                         ('Keep largest component', lambda: self.geometry('filter')),
                         ('Normalize', lambda: self.geometry('normalize')),
                         ('Use output as next input', lambda: self.variables['mesh_input'].set(self.get('mesh_output')))])
        self.section(p, 'Generate or scale the surface job', 'Projection axis and U/V axes must be distinct. Patch limits and spacing use the input mesh units. The exported CSV is in the object frame.')
        self.field(p, 'projection', 'Projection axis', 'z', choices=('x','y','z'))
        self.field(p, 'side', 'Projection side', 'positive', choices=('positive','negative'))
        self.field(p, 'u_axis', 'U axis', 'x', choices=('x','y','z'))
        self.field(p, 'v_axis', 'V axis', 'y', choices=('x','y','z'))
        for key,label,default in [('u_min','U minimum','-20'),('u_max','U maximum','20'),
                                  ('v_min','V minimum','-20'),('v_max','V maximum','20'),
                                  ('angle','Raster angle (degrees)','0'),('point_spacing','Point spacing (mm)','1'),
                                  ('line_spacing','Line spacing (mm)','5'),('standoff','Standoff (mm)','2')]:
            self.field(p,key,label,default)
        self.field(p, 'object_csv', 'Object-mm raster CSV', '/tmp/object_raster.csv', browse='save')
        self.field(p, 'scale', 'Uniform scale factor', '1.5')
        self.field(p, 'scaled_csv', 'Scaled CSV', '/tmp/object_raster_scaled.csv', browse='save')
        self.buttons(p, [('Generate raster', lambda: self.geometry('raster')),
                         ('Validate raster', lambda: self.geometry('validate')),
                         ('Scale mesh + raster', lambda: self.geometry('scale')),
                         ('Generate test saddle', lambda: self.geometry('saddle'))])

    def geometry(self, operation):
        repo = Path(self.get('pipeline')).expanduser().resolve()
        if not (repo/'pyproject.toml').is_file():
            raise ValueError('Select the offline geometry repository on the Session tab')
        scripts={'inspect':'inspect_mesh.py','prepare':'prepare_mesh.py','filter':'filter_mesh_components.py',
                 'normalize':'normalize_mesh_frame.py','raster':'generate_projected_raster.py',
                 'validate':'validate_toolpath.py','scale':'scale_surface_job.py','saddle':'generate_saddle_stl.py'}
        script=repo/scripts[operation]
        if not script.is_file():
            raise ValueError(f'Missing geometry script: {script}')
        argv=['uv','run','--project',str(repo),'python',str(script)]
        if operation=='saddle':
            argv+=['--output',path(self.get('mesh_output'),'Output mesh')]
            if Path(self.get('mesh_output')).suffix.lower()!='.stl':
                raise ValueError('Test saddle output must have .stl extension')
        elif operation=='validate':
            argv+=['--input',path(self.get('object_csv'),'Object CSV',True),
                   '--expected-standoff',str(number(self.get('standoff'),'Standoff',0)),
                   '--max-point-step',str(2*number(self.get('point_spacing'),'Spacing',0.001))]
        else:
            source=path(self.get('mesh_input'),'Input mesh',True)
            if operation=='inspect': argv+=['--mesh',source,'--skip-topology']
            elif operation in ('prepare','filter','normalize'):
                output=path(self.get('mesh_output'),'Output mesh')
                if output==source: raise ValueError('Use a separate output mesh')
                if Path(output).exists() and not messagebox.askyesno('Replace output?', f'Replace {output}?'): return
                argv+=['--input',source,'--output',output]
                if operation=='prepare': argv+=['--orient-triangles','--overwrite']
                if operation=='filter': argv+=['--keep-largest','1']
                if operation=='normalize': argv+=['--source-up='+self.get('source_up'),'--units','mm',
                                                  '--transform-output',path(self.get('normalization'),'Transform JSON')]
            elif operation=='raster':
                axes=[self.get(k) for k in ('projection','u_axis','v_axis')]
                if len(set(axes))!=3: raise ValueError('Projection, U and V axes must be distinct')
                if number(self.get('u_min'),'U min')>=number(self.get('u_max'),'U max') or number(self.get('v_min'),'V min')>=number(self.get('v_max'),'V max'):
                    raise ValueError('Patch minimum must be less than maximum')
                argv+=['--mesh',source,'--output-csv',path(self.get('object_csv'),'Object CSV'),
                       '--projection-axis',axes[0],'--projection-side',self.get('side'),
                       '--u-axis',axes[1],'--v-axis',axes[2]]
                for key,flag,low in [('u_min','--u-min',None),('u_max','--u-max',None),
                                     ('v_min','--v-min',None),('v_max','--v-max',None),
                                     ('angle','--raster-angle-deg',None),('point_spacing','--point-spacing',0.001),
                                     ('line_spacing','--line-spacing',0.001),('standoff','--standoff',0)]:
                    argv+=[flag,str(number(self.get(key),key,low))]
            elif operation=='scale':
                output=path(self.get('mesh_output'),'Output mesh')
                raw=path(self.get('object_csv'),'Object CSV',True)
                scaled=path(self.get('scaled_csv'),'Scaled CSV')
                if source==output or raw==scaled: raise ValueError('Scale outputs must differ from inputs')
                argv+=['--input-mesh',source,'--input-toolpath',raw,'--output-mesh',output,
                       '--output-toolpath',scaled,'--scale-factor',str(number(self.get('scale'),'Scale',0.001)),
                       '--center-mode','origin','--standoff-mm',str(number(self.get('standoff'),'Standoff',0))]
        self.run('Geometry '+operation,argv)

    def path_page(self):
        p=self.pages['3  Path']
        self.section(p,'Capture at the current nozzle pose','Capturing writes a path without moving the robot. Tube capture needs a downward-facing nozzle; fixed orientation, world XY helix rising in +Z.')
        self.field(p,'shape','Shape','line',choices=('line','circle','square','tube'))
        self.field(p,'size','Line size / circle or tube diameter (mm)','5')
        self.field(p,'plane','Planar shape plane','xy',choices=('xy','xz','yz'))
        self.field(p,'direction','Planar direction','-1',choices=('-1','1'))
        self.field(p,'height','Tube height (mm)','1')
        self.field(p,'pitch','Tube pitch (mm)','0.5')
        self.field(p,'tube_step','Tube point spacing (mm)','0.25')
        self.field(p,'world_csv','World-meter path CSV','/tmp/demo_path.csv',browse='save')
        self.buttons(p,[('Capture path',self.capture_path)])
        self.section(p,'Register an offline object path','Input is the object-mm CSV from Geometry. Enter a measured normalized-object-to-world pose. Conversion is applied once and does not locate the object.')
        for key,label,default in [('object_x','Object X (m)','0.55'),('object_y','Object Y (m)','-0.25'),
                                  ('object_z','Object Z (m)','0.10'),('object_roll','Roll (degrees)','0'),
                                  ('object_pitch','Pitch (degrees)','0'),('object_yaw','Yaw (degrees)','0')]:
            self.field(p,key,label,default)
        self.buttons(p,[('Convert object CSV > world CSV',self.convert_path)])

    def capture_path(self):
        self.invalidate()
        self.run('Capture path',capture(self.session(),self.get('shape'),self.get('size'),self.get('plane'),
                 self.get('direction'),self.get('world_csv'),self.get('height'),self.get('pitch'),self.get('tube_step')))

    def convert_path(self):
        self.invalidate()
        self.run('Register CSV',convert(self.get('object_csv'),self.get('world_csv'),
                 [self.get(k) for k in ('object_x','object_y','object_z')],
                 [self.get(k) for k in ('object_roll','object_pitch','object_yaw')]))

    def print_page(self):
        p=self.pages['4  Plan & Print']
        self.section(p,'Plan one registered line','Uses world-meter CSV from Path. Position the nozzle at its beginning, add collisions in RViz, then plan. No automatic approach, retract or return to start is added.')
        for key,label,default in [('line','Line ID','0'),('speed','TCP speed (mm/s)','1'),
                                  ('velocity','Velocity scaling','0.01'),('acceleration','Acceleration scaling','0.01'),
                                  ('eef_step','EEF step (m)','0.001')]:self.field(p,key,label,default)
        self.field(p,'trajectory','Saved plan YAML','/tmp/surface_line_plan.yaml',browse='save')
        self.buttons(p,[('1. Plan + publish RViz preview',self.plan_path),('2. Check saved plan (no motion)',self.check_plan)])
        self.section(p,'Rehearse or print the checked plan','Inspect the complete RViz trajectory before proceeding. Execution repeats model/start/scene/speed checks. A completed rehearsal consumes this plan; return to start and replan.')
        self.field(p,'step_rate_hz','Target pulse frequency (Hz; 0 selects legacy flow)','200')
        self.field(p,'max_pulses','Optional pulse budget (0 = no count cap)','0')
        self.field(p,'flow','Legacy flow (mL/min; used only at 0 Hz)','1')
        self.field(p,'duration','Duration budget (s, <=100)','60')
        self.field(p,'volume','Legacy volume budget (mL; used only at 0 Hz)','1')
        self.checkbox(p,'I reviewed the full RViz path, calibration, scene, controller and physical stop setup',self.confirmed)
        self.checkbox(p,'Enable physical extrusion during robot motion',self.extrude)
        self.execute_button=self.buttons(p,[('3. Execute checked plan',self.execute_plan)])[0]
        self.execute_button.configure(state='disabled')
        self.confirmed.trace_add('write',self.update_execute_button)
        self.extrude.trace_add('write',self.update_execute_button)
        self.job_fields=('mode','domain','workspace','world_csv','trajectory','line','speed','velocity',
                         'acceleration','eef_step','flow','duration','volume','step_rate_hz','max_pulses')

    def job_key(self):
        values=tuple(self.get(k) for k in self.job_fields)
        csv=Path(self.get('world_csv')).expanduser()
        fingerprint=hashlib.sha256(csv.read_bytes()).hexdigest() if csv.is_file() else None
        return values,fingerprint

    def signature(self):
        p=Path(self.get('trajectory')).expanduser()
        if not p.is_file(): return None
        return self.job_key(),hashlib.sha256(p.read_bytes()).hexdigest()

    def plan_path(self):
        self.invalidate()
        submitted=self.job_key()
        def finished(code):
            success=code==0 and submitted==self.job_key()
            self.gate.plan_finished(self.signature(),success)
            self.plan_state.set('Planned | inspect in RViz, then run check-only' if success else 'Plan failed or job changed | execute is locked')
        argv=plan(self.get('world_csv'),self.get('trajectory'),self.get('line'),self.get('speed'),
                  self.get('velocity'),self.get('acceleration'),self.get('eef_step'))
        self.run('Plan path',argv,callback=finished)

    def check_plan(self):
        sig=self.signature()
        if sig is None or sig!=self.gate.planned:
            raise ValueError('Create a fresh successful plan for this exact job first')
        def finished(code):
            success=code==0 and sig==self.signature()
            self.gate.check_finished(sig,success)
            self.update_execute_button()
            self.plan_state.set('Checked | review RViz and acknowledge before execution' if self.gate.ready(sig) else 'Check failed or job changed | execute is locked')
        self.gate.checked = None
        self.update_execute_button()
        self.run('Check plan',self.execution_args(False),callback=finished)

    def execution_args(self,execute):
        return execution(self.session(),self.get('trajectory'),execute,
                         self.extrude.get() if execute else False,self.confirmed.get(),
                         self.get('flow'),self.get('duration'),self.get('volume'),
                         self.get('step_rate_hz'),self.get('max_pulses'))

    def execute_plan(self):
        if not self.gate.ready(self.signature()):
            raise ValueError('Run Plan and Check successfully for this exact job before executing')
        if not self.confirmed.get():raise ValueError('Review RViz and acknowledge the checklist first')
        if self.session().simulation and not self.mock_isolated.get():
            raise ValueError('Confirm mock isolation on the Session tab')
        argv=self.execution_args(True)
        self.run('Execute plan',argv,kind='motion')
        self.gate.invalidate()
        self.confirmed.set(False)
        self.extrude.set(False)
        self.plan_state.set('Execution requested | this plan is consumed; replan before the next run')

    def pump_page(self):
        p=self.pages['5  Syringe']
        self.section(p,'Bounded syringe dispense / retract','Physical mode only. Counts estimate delivery; no encoder, pressure or cumulative stroke sensor is available. Negative volume retracts. The existing backend enforces ramp, pulse/time limits and exclusive access.')
        self.field(p,'pump_volume','Signed volume (mL)','0.1')
        self.field(p,'pump_flow','Flow magnitude (mL/min)','1')
        self.checkbox(p,'I verified direction, free stroke, clear nozzle and independent pump power stop',self.pump_confirmed)
        self.buttons(p,[('Dispense / retract',self.dispense_pump),('Pump STOP',lambda:self.request_stop(pump_only=True))])
        self.section(p,'Firmware and tool configuration','Use the firmware guide for serial tests/watchdog controls. The console does not disable watchdogs or change calibration flags. Do not open Serial Monitor while ROS owns the port.')
        self.buttons(p,[('Read current syringe status',lambda:self.run('Syringe status',['ros2','topic','echo','/syringe/status','--once'])),
                         ('Inspect tool0 > tip TF',lambda:self.run('TCP transform',['ros2','run','tf2_ros','tf2_echo','tool0','probe_tcp']))])

    def dispense_pump(self):
        argv=dispense(self.session(),self.get('pump_volume'),self.get('pump_flow'),self.pump_confirmed.get())
        self.run('Pump dispense',argv,kind='motion')
        self.pump_confirmed.set(False)

    def diagnostics_page(self):
        p=self.pages['6  Diagnostics']
        self.section(p,'Inspect the live system','These commands report state. Long-running TF/topic commands can be interrupted in the Session process list.')
        self.buttons(p,[('Nodes',lambda:self.run('Nodes',['ros2','node','list'])),
                         ('Controllers',lambda:self.run('Controllers',['ros2','control','list_controllers'])),
                         ('Joint state',lambda:self.run('Joints',['ros2','topic','echo','/joint_states','--once'])),
                         ('Topics',lambda:self.run('Topics',['ros2','topic','list']))])
        self.section(p,'Point-cloud processing','Uses existing default cloud topics. Centroid/bounds are not object pose registration. Camera acquisition is optional and must match the actual topic namespace.')
        self.buttons(p,[('Launch RealSense',lambda:self.run('RealSense',['ros2','launch','realsense2_camera','rs_launch.py','pointcloud.enable:=true','align_depth.enable:=true'],background=True)),
                         ('Start cloud preprocessing',lambda:self.run('Cloud preprocessing',ros_run('ur5e_perception','pointcloud_preprocessor'),background=True)),
                         ('Start segmentation',lambda:self.run('Cloud segmentation',ros_run('ur5e_perception','table_object_segmenter'),background=True)),
                         ('Start measurement',lambda:self.run('Cloud measurement',ros_run('ur5e_perception','object_measurement'),background=True))])
        self.section(p,'Scope of this first GUI release','C++ multi-line saddle execution, automatic registration, embedded 3D viewing, firmware flashing and calibration editing remain separate workflows. This console exposes the bounded Python planning/execution route first.')

    def run(self,name,argv,background=False,kind='command',callback=None,session=None,stop=False):
        s=session or self.session()
        if not stop:
            if self.task is not None:
                raise ValueError('A workflow step is running. Wait or interrupt it before starting another.')
            for job in self.processes.jobs.values():
                if job.kind=='background' and job.session!=s:
                    raise ValueError('Stop owned background processes before changing mode, domain or workspace')
            if kind=='motion' and self.processes.moving():
                raise ValueError('Another motion operation is running')
            if self.processes.moving():raise ValueError('Wait for motion to stop before starting another workflow')
        if not shutil.which(argv[0]):raise ValueError(f'{argv[0]} is not on PATH; source ROS or install uv as appropriate')
        if argv[0] == 'ros2' and not stop:
            import os
            expected = Path(s.workspace) / 'install'
            prefixes = [Path(v).resolve() for v in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if v]
            if not any(prefix == expected or expected in prefix.parents for prefix in prefixes):
                raise ValueError(f'The selected workspace is not sourced. Close the GUI, source {expected}/setup.bash, then relaunch.')
        self.log(name,f'[domain {s.domain}, {s.mode}] '+shlex.join(argv))
        self.processes.start(name,argv,s,'background' if background else kind)
        if self.mock_window and self.mock_window.session == s and not stop:
            self.mock_window.started_job(name)
        if not background and not stop:self.task=name
        if callback:self.callbacks[name]=callback
        self.refresh_jobs()
        self.status.set(f'{s.mode.upper()} | {name}')
        self.update_execute_button()

    def request_stop(self,pump_only=False,session=None):
        # Dispatch to original motion domains even if settings were edited mid-run.
        moving=[job for job in self.processes.moving() if session is None or job.session == session]
        try:
            sessions=set(job.session for job in moving) or {session or self.session()}
            for s in sessions:
                for argv in (stop_commands()[1:] if pump_only else stop_commands()):
                    self.sequence+=1
                    self.run(f'STOP request {self.sequence}', ['timeout', '--signal=INT', '--kill-after=2s', '8s', *argv], session=s, stop=True)
            for job in moving:
                if not pump_only or job.name=='Pump dispense':self.processes.interrupt(job.name)
            self.log('STOP','Requests sent; watch hardware and acknowledgements. Use physical stops if motion continues.')
        except (ValueError,OSError) as exc:
            messagebox.showerror('Stop request could not be sent', str(exc)+'\nUse the physical robot and independent pump stops.')

    def interrupt_selected(self):
        selected=self.job_list.curselection()
        if not selected:return
        name=self.job_list.get(selected[0])
        job=self.processes.jobs.get(name)
        if job and job.kind=='motion':self.request_stop()
        elif job:
            if name in ('Robot stack','Syringe controller'):
                if self.processes.moving():raise ValueError('Stop active motion before shutting down a controller')
                if not messagebox.askyesno('Shut down controller?',f'Interrupt {name}? Confirm robot and pump are stopped.'):return
                self.invalidate()
            self.processes.interrupt(name)

    def refresh_jobs(self):
        self.job_list.delete(0,'end')
        for name in self.processes.jobs:self.job_list.insert('end',name)

    def log(self,name,text):
        if not hasattr(self,'logs'):return
        text=re.sub(r'\x1b\[[0-9;]*[A-Za-z]','',text)
        self.logs.configure(state='normal')
        self.logs.insert('end',f'[{name}] {text}\n')
        lines=int(self.logs.index('end-1c').split('.')[0])
        if lines>1500:self.logs.delete('1.0',f'{lines-1500}.0')
        self.logs.see('end')
        self.logs.configure(state='disabled')

    def poll(self):
        # Bound work per tick so chatty ROS output cannot starve stop-button events.
        for _ in range(200):
            try:event,name,value=self.processes.events.get_nowait()
            except queue.Empty:break
            if event=='log':
                if name == 'Mock telemetry' and value.startswith('MOCK_STATE '):
                    if self.mock_window:
                        try:self.mock_window.feedback.accept(value)
                        except (ValueError, TypeError, json.JSONDecodeError) as exc:self.log(name, str(exc))
                else:
                    self.log(name,value)
                    job=self.processes.jobs.get(name)
                    if self.mock_window and job and job.session == self.mock_window.session:
                        self.mock_window.append(name,value)
            else:
                self.log(name,f'Exited with code {value}')
                job=self.processes.reap(name)
                if self.task==name:self.task=None
                callback=self.callbacks.pop(name,None)
                if callback:
                    try:callback(value)
                    except (ValueError,OSError) as exc:self.log('GUI',str(exc));self.invalidate()
                if self.mock_window and job and job.session == self.mock_window.session:
                    self.mock_window.finished_job(name,value)
                if job and job.kind=='background':self.invalidate()
                if name.startswith('STOP request'):self.invalidate()
                self.refresh_jobs()
                self.update_execute_button()
                self.status.set(self.get('mode').upper()+' | Ready' if not self.task else self.task)
        self.poll_after = self.root.after(100,self.poll)

    def close(self):
        if self.processes.moving():
            self.request_stop()
            messagebox.showinfo('Stop requested','The console stays open while a motion process is running. Confirm physical cessation and wait for the process to exit before closing.')
            return
        if self.processes.jobs:
            messagebox.showinfo('Background processes still running','Interrupt owned processes in the Session tab before closing. The console will not silently abandon robot/controller processes.')
            return
        if self.mock_window:
            self.mock_window.close()
        self.root.after_cancel(self.poll_after)
        self.root.destroy()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path.home()/'ur5e_ws')
    parser.add_argument('--pipeline',type=Path,default=Path.home()/'Surface_printing_path_generation_pipeline')
    args=parser.parse_args()
    try:
        root=tk.Tk()
    except tk.TclError as exc:
        raise SystemExit('A graphical desktop is required. On Ubuntu install python3-tk and launch from the desktop session: '+str(exc))
    PrintingApp(root,str(args.workspace),str(args.pipeline))
    root.mainloop()


if __name__=='__main__':
    main()
