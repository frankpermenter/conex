"""
Thin Python shim for the compiled pybind11 extension.

The extension module is named ``_conex`` and is built by CMake.
"""

try:
    from _conex import *  # noqa: F401,F403
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "Could not import '_conex'. Build the extension first with "
        "'make -C interfaces/python' (requires CMake)."
    ) from exc
