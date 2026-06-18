#!/usr/bin/env python3
"""
Single entry point: prep -> train -> convert -> deploy -> compare.

  python3 run_pipeline.py                  # run the whole chain
  python3 run_pipeline.py --start convert  # resume from a stage
  python3 run_pipeline.py --only compare   # run one stage

Stages share state through <workdir> (manifest, checkpoints, ONNX, cvimodel,
predictions). The board is contacted only in the `deploy` stage.
"""
import sys
import argparse
import subprocess

STAGES = ["prep", "train", "convert", "deploy", "compare"]
SCRIPT = {
    "prep": "data_prep.py",
    "train": "train.py",
    "convert": "convert.py",
    "deploy": "deploy_eval.py",
    "compare": "compare.py",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--start", choices=STAGES, help="start from this stage")
    ap.add_argument("--only", choices=STAGES, help="run only this stage")
    args = ap.parse_args()

    if args.only:
        stages = [args.only]
    elif args.start:
        stages = STAGES[STAGES.index(args.start):]
    else:
        stages = STAGES

    for st in stages:
        print(f"\n========== STAGE: {st} ==========")
        r = subprocess.run([sys.executable, SCRIPT[st], "--config", args.config])
        if r.returncode != 0:
            raise SystemExit(f"[run_pipeline] stage '{st}' failed (rc={r.returncode})")
    print("\n[run_pipeline] pipeline complete.")


if __name__ == "__main__":
    main()
