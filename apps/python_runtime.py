"""Select the installed interpreter before an application imports dependencies."""
import os
from pathlib import Path
import platform
import shutil
import sys


def ensure_runtime(script, *, camera=False, simulation=False):
    """Re-exec a direct Python entry point without requiring shell activation."""
    script = Path(script).resolve()
    root = script.parents[2]
    persistent = Path(os.environ.get('G1_DEPS_DIR') or
                      Path.home() / '.local/share/g1-deploy' / platform.machine())
    dependency_roots = [persistent, root / '.deps', root.parent / '.deps']
    override = os.environ.get('G1_SIM_PYTHON') if simulation else None
    override = override or os.environ.get('PYTHON_BIN')
    interpreter = None
    if override:
        interpreter = shutil.which(override)
        if interpreter is None:
            raise SystemExit(f'Python interpreter not found: {override}')
    elif simulation:
        # The external simulator owns its MuJoCo/GUI dependencies. Discover its
        # usual virtual environments without activating them in the user's shell.
        for index, argument in enumerate(sys.argv[1:], 1):
            if argument == '--mujoco-root' and index + 1 < len(sys.argv):
                simulator = Path(sys.argv[index + 1]).expanduser()
                break
            if argument.startswith('--mujoco-root='):
                simulator = Path(argument.split('=', 1)[1]).expanduser()
                break
        else:
            simulator = None
        if simulator is not None:
            for name in ('.venv', 'venv'):
                candidate = simulator / name / 'bin/python3'
                if os.access(candidate, os.X_OK):
                    interpreter = str(candidate.absolute())
                    break
    else:
        for directory in dependency_roots:
            candidate = directory / 'python/bin/python3'
            if os.access(candidate, os.X_OK):
                interpreter = str(candidate.absolute())
                wrapper = directory / 'python/bin/python3-realsense'
                if camera and os.access(wrapper, os.X_OK):
                    interpreter = str(wrapper.absolute())
                break
    interpreter = interpreter or sys.executable
    identity = f'{script}:{interpreter}'
    if os.environ.get('_G1_PYTHON_RUNTIME') == identity:
        # Do not pass the one-time re-exec marker to child applications.
        os.environ.pop('_G1_PYTHON_RUNTIME', None)
        return

    env = os.environ.copy()
    env['_G1_PYTHON_RUNTIME'] = identity
    env['PATH'] = str(Path(interpreter).parent) + os.pathsep + env.get('PATH', '')
    # PYTHONHOME from another shell environment must not override a selected venv.
    env.pop('PYTHONHOME', None)
    libraries = []
    for directory in dependency_roots:
        if (directory / 'cyclonedds/lib/libddsc.so').is_file():
            libraries.append(str(directory / 'cyclonedds/lib'))
            break
    if camera:
        for directory in dependency_roots:
            if (directory / 'realsense/lib').is_dir():
                libraries.append(str(directory / 'realsense/lib'))
                break
    if libraries:
        env['LD_LIBRARY_PATH'] = os.pathsep.join(libraries +
            ([env['LD_LIBRARY_PATH']] if env.get('LD_LIBRARY_PATH') else []))
    local_packages = root / '.python-deps'
    if local_packages.is_dir():
        env['PYTHONPATH'] = str(local_packages) + (
            os.pathsep + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    os.execve(interpreter, [interpreter, '-u', str(script), *sys.argv[1:]], env)
