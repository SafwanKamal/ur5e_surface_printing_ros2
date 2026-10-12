"""Validated argv builders. No shell, ROS imports, or implicit physical defaults."""
from dataclasses import dataclass
import math
from pathlib import Path
import os


def number(value, name, low=None, high=None):
    try:
        value = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f'{name} must be numeric') from exc
    if not math.isfinite(value) or (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f'{name} must be finite and within {low}..{high}')
    return value


def integer(value, name, low, high):
    result = number(value, name, low, high)
    if not result.is_integer():
        raise ValueError(f'{name} must be an integer')
    return int(result)


def path(value, name, exists=False):
    if not str(value).strip():
        raise ValueError(f'{name} is required')
    result = Path(value).expanduser().resolve()
    if exists and not result.is_file():
        raise ValueError(f'{name} does not exist: {result}')
    return str(result)


def ros_run(package, executable, **parameters):
    argv = ['ros2', 'run', package, executable]
    if parameters:
        argv.append('--ros-args')
        for name, value in parameters.items():
            if isinstance(value, bool):
                value = str(value).lower()
            argv.extend(['-p', f'{name}:={value}'])
    return argv


@dataclass(frozen=True)
class Session:
    mode: str
    domain: int
    workspace: str

    def __post_init__(self):
        if self.mode not in ('mock', 'physical'):
            raise ValueError('Choose mock or physical mode')
        integer(self.domain, 'ROS domain', 0, 232)
        if not Path(self.workspace).expanduser().is_dir():
            raise ValueError('Workspace must be an existing directory')

    @property
    def simulation(self):
        return self.mode == 'mock'

    def environment(self):
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self.domain)
        return env


def capture(session, shape, size, plane, direction, output, height, pitch, step):
    output = path(output, 'Path CSV')
    if shape == 'tube':
        if number(pitch, 'Pitch', 0.01, 100) > number(height, 'Height', 0.01, 100):
            raise ValueError('Tube pitch cannot exceed height')
        return ros_run('surface_printing', 'make_tube_path',
                       simulation=session.simulation, diameter_mm=number(size, 'Diameter', 0.1, 100),
                       height_mm=number(height, 'Height', 0.01, 100),
                       pitch_mm=number(pitch, 'Pitch', 0.01, 100),
                       step_mm=number(step, 'Point spacing', 0.01, 1), output_path=output)
    if shape not in ('line', 'circle', 'square') or plane not in ('xy', 'xz', 'yz'):
        raise ValueError('Unsupported shape or plane')
    direction = number(direction, 'Direction')
    if direction not in (-1, 1):
        raise ValueError('Direction must be -1 or +1')
    return ros_run('surface_printing', 'make_demo_path', shape=shape,
                   size_mm=number(size, 'Size', 0.1, 100), plane=plane, direction=direction,
                   frame_id='world', link_name='probe_tcp', output_path=output)


def plan(csv, output, line, speed, velocity, acceleration, step):
    return ros_run('surface_printing', 'plan_surface_path',
                   csv_path=path(csv, 'World-meter CSV', True),
                   output_path=path(output, 'Saved plan'), frame_id='world',
                   link_name='probe_tcp', group_name='ur_manipulator',
                   line_id=integer(line, 'Line ID', 0, 100000),
                   cartesian_speed_mm_s=number(speed, 'TCP speed', 0.1, 10),
                   velocity_scale=number(velocity, 'Velocity scale', 0.0001, 1),
                   acceleration_scale=number(acceleration, 'Acceleration scale', 0.0001, 1),
                   eef_step=number(step, 'EEF step', 0.00001, 0.01))


def execution(session, trajectory, execute, extrude, confirmed, flow, seconds, volume,
              step_rate_hz=0, max_pulses=0):
    if extrude and session.simulation:
        raise ValueError('Mock mode forbids extrusion')
    if execute and not session.simulation and not confirmed:
        raise ValueError('Physical motion requires the hardware checklist acknowledgement')
    rate = integer(step_rate_hz, 'Target pulse frequency', 0, 5000)
    settings = (dict(step_rate_hz=float(rate), extrusion_direction=-1,
                     max_pulses=integer(max_pulses, 'Pulse budget', 0, 2147483647)) if rate else
                dict(step_rate_hz=0.0, flow_ml_per_min=number(flow, 'Flow', 0.001, 5),
                     max_volume_ml=number(volume, 'Volume budget', 0.001, 5)))
    return ros_run('surface_printing', 'execute_surface_print',
                   trajectory_path=path(trajectory, 'Saved plan', True),
                   simulation=session.simulation, execute=execute, extrude=extrude,
                   hardware_confirmed=bool(confirmed and not session.simulation),
                   max_duration_sec=number(seconds, 'Duration budget', 0.01, 100),
                   **settings)


def convert(source, output, xyz, rpy):
    source, output = path(source, 'Object-mm CSV', True), path(output, 'World CSV')
    if source == output:
        raise ValueError('Conversion output must differ from input')
    return ['ros2', 'run', 'surface_printing', 'convert_surface_csv', source, output,
            '--units', 'mm', '--xyz', *[str(number(v, 'Object translation')) for v in xyz],
            '--rpy-deg', *[str(number(v, 'Object rotation')) for v in rpy]]


def dispense(session, volume, flow, acknowledged):
    if session.simulation:
        raise ValueError('Pump motion is disabled in mock mode')
    if not acknowledged:
        raise ValueError('Acknowledge direction, free travel and clear nozzle before pump motion')
    volume = number(volume, 'Dispense volume', -5, 5)
    if volume == 0:
        raise ValueError('Dispense volume cannot be zero')
    flow = number(flow, 'Dispense flow', 0.001, 5)
    return ['ros2', 'action', 'send_goal', '/syringe/dispense_volume',
            'syringe_interfaces/action/DispenseVolume',
            f'{{volume_ml: {volume}, flow_ml_per_min: {flow}}}', '--feedback']


def stop_commands():
    return [['ros2', 'service', 'call', name, 'std_srvs/srv/Trigger', '{}']
            for name in ('/surface_printing/stop', '/syringe/stop')]


class PlanGate:
    """Execution is enabled only after a fresh successful check of this exact job."""
    def __init__(self):
        self.planned = None
        self.checked = None

    def invalidate(self):
        self.planned = self.checked = None

    def plan_finished(self, signature, success):
        self.planned = signature if success else None
        self.checked = None

    def check_finished(self, signature, success):
        self.checked = signature if success and signature == self.planned else None

    def ready(self, signature):
        return signature == self.planned == self.checked and signature is not None
