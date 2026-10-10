"""Resolve bundled sources from this project, never from a fixed home path."""
import os
from pathlib import Path


def project_root(start=None):
    path=Path(start or __file__).resolve()
    candidates=([path] if path.is_dir() else [])+list(path.parents)
    # A stale environment from another checkout cannot override this source tree.
    override=os.environ.get('ROBOT_PROJECT_ROOT')
    if override:candidates.append(Path(override).expanduser().resolve())
    for candidate in candidates:
        if (candidate/'.7503robot-project').is_file() and (candidate/'navigation/scripts/scene.py').is_file():
            return candidate
    raise RuntimeError('找不到完整 7503robot-main 项目：请保留 navigation、src 和 .7503robot-project，换目录后重新构建')


def navigation_root(start=None):
    return project_root(start)/'navigation'
