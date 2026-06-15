"""Shared config loading + workdir helpers."""
import os

try:
    import yaml
except ImportError as e:
    raise SystemExit("PyYAML required: pip install pyyaml") from e


def load_config(path="config.yaml"):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return cfg


def ensure_workdir(cfg):
    here = os.path.dirname(os.path.abspath(__file__))
    workdir = cfg["paths"]["workdir"]
    if not os.path.isabs(workdir):
        workdir = os.path.join(here, workdir)
    os.makedirs(workdir, exist_ok=True)
    return workdir
