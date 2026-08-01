"""Compatibility entry point for the OpenFOAM-based aero-drag tool.

The real implementation lives in the modular CLI.
"""

from optimise_hull import main


if __name__ == "__main__":
    raise SystemExit(main())
