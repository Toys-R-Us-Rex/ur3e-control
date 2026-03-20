
from src.logger import DataStore


def run_stage(stage_obj, on_error="stop"):
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

class Stage:
    def __init__(self, name, datastore: DataStore):
        self.name = name
        self.ds = datastore

    def run(self):
        raise NotImplementedError

    def fallback(self):
        raise NotImplementedError