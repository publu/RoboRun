"""Optional packages: the base install of roborun-sdk is small, and the heavy or hardware packages come with extras.

    pip install roborun-sdk            the client, your own drivers and rules (StreamedArm, ArmAdapter), the `roborun` command
    pip install 'roborun-sdk[robot]'   everything a robot runs (what install.sh installs): simulator, cameras, arms, link

`lazy('numpy', 'cameras')` stands in for a module and imports it on first use, so importing a Roborun module never pulls
numpy, MuJoCo or OpenCV until something needs them. A package that is missing then fails with MissingExtra, which names
the extra to install (it is a ModuleNotFoundError, so `except ImportError` still catches it).
"""
import importlib

# the extra that brings each optional top-level module; anything else (numpy, Pillow, the CAN and serial packages, the YAM
# driver's packages) is in [robot], which has all of them
EXTRA_FOR = {
    'mujoco': 'sim',
    'cv2': 'cameras', 'pyrealsense2': 'cameras',
    'msgpack': 'link', 'websockets': 'link', 'zstandard': 'link',
    'cryptography': 'crypto',
    'pyarrow': 'dataset', 'av': 'dataset',
    'numpy': 'robot', 'PIL': 'robot', 'can': 'robot', 'serial': 'robot', 'usb': 'robot', 'libusb_package': 'robot',
    'tyro': 'robot', 'dm_env': 'robot', 'mcap': 'robot', 'mcap_ros2': 'robot', 'crcmod': 'robot', 'pydantic': 'robot',
}


class MissingExtra(ModuleNotFoundError):
    """An optional package is not installed. The message names the roborun-sdk extra that brings it."""


def install_hint(module, extra=None) -> str:
    top = str(module or '').split('.')[0]
    extra = extra or EXTRA_FOR.get(top) or 'robot'
    also = '' if extra == 'robot' else ", or 'roborun-sdk[robot]' for everything a robot runs"
    return f"{top} is not installed: pip install 'roborun-sdk[{extra}]'{also}"


def require(module, extra=None):
    """import_module(module), or MissingExtra naming the extra to install."""
    try:
        return importlib.import_module(module)
    except ModuleNotFoundError as exc:
        if exc.name and module.split('.')[0] != exc.name.split('.')[0]:
            raise                                          # the package is there, but something it needs is not
        raise MissingExtra(install_hint(module, extra), name=exc.name) from exc


def explain(exc):
    """The install hint for a ModuleNotFoundError about one of Roborun's optional packages, else None."""
    if isinstance(exc, MissingExtra):
        return str(exc)
    top = (getattr(exc, 'name', None) or '').split('.')[0]
    return install_hint(top) if top in EXTRA_FOR else None


class lazy:
    """A module imported on first attribute access: `np = lazy('numpy', 'cameras', globals())`, then `np.zeros(...)` as
    usual. Given the module's globals, the first use also puts the real module in place of this stand-in there, so later
    uses cost nothing extra."""
    __slots__ = ('_name', '_extra', '_module', '_namespace')

    def __init__(self, name, extra=None, namespace=None):
        self._name, self._extra, self._module, self._namespace = name, extra, None, namespace

    def _load(self):
        if self._module is None:
            self._module = require(self._name, self._extra)
            for key, value in list((self._namespace or {}).items()):
                if value is self:
                    self._namespace[key] = self._module
        return self._module

    def __getattr__(self, attr):
        return getattr(self._load(), attr)

    def __repr__(self):
        return f'<lazy module {self._name!r}{"" if self._module is None else " (loaded)"}>'
