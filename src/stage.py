"""
Stage Module
============

This module provides the basic structure for defining stages in the calibration pipeline.


MIT License

Copyright (c) 2026 Mariéthoz Cédric

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author:     Mariéthoz Cédric, with assistance from Copilot AI (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304
"""

from src.logger import DataStore


class Stage:
    """
    Basic class structure for a stage in the pipeline.
    """
    def __init__(self, name, datastore: DataStore):
        self.name = name
        self.ds = datastore

    def run(self):
        raise NotImplementedError

    def fallback(self):
        raise NotImplementedError


def run_stage(stage_obj: Stage, on_error="stop"):
    """
    Run a stage with error handling.

    The error handling could be interpreted in different ways depending on the value of `on_error`:
    - "stop": Stop the pipeline if an error occurs.
    - "continue": Continue to the next stage even if an error occurs.
    - "fallback": Attempt the fallback action of the stage if an error occurs.

    Parameters
    ----------
    stage_obj : Stage
        The stage object to run.
    on_error : str, optional
        Error handling strategy. Options are "stop", "continue", or "fallback".
    """
    stage_obj.ds.log(f"Starting stage: {stage_obj.name}")

    try:
        stage_obj.run()
        stage_obj.ds.log(f"Stage completed: {stage_obj.name}")

    except Exception as e:
        stage_obj.ds.log(f"ERROR in {stage_obj.name}: {e}")

        if on_error == "continue":
            print(f"Continuing despite error in {stage_obj.name}:\n{e}")
            stage_obj.ds.log(f"Continuing despite error in {stage_obj.name}")
            return

        if on_error == "fallback":
            print(f"Attempting fallback for {stage_obj.name}:\n{e}")
            stage_obj.ds.log(f"Attempting fallback for {stage_obj.name}")
            stage_obj.fallback()
            return

        if on_error == "stop":
            stage_obj.ds.log(f"Stopping pipeline due to error in {stage_obj.name}")
            raise