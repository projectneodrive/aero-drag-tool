"""Compatibility entry point for the aero-drag tool.

This keeps the original filename working while the real implementation
lives in the modular CLI.
"""

from optimise_hull import main


if __name__ == "__main__":
    raise SystemExit(main())