import argparse
from colcon_core.verb import VerbExtensionPoint
from colcon_core.command import add_subparsers, CommandContext
from subprocess import run
import os

class IdfWrapper(VerbExtensionPoint):
    def __init__(self) -> None:
        super().__init__()
        self.package = ""
        self.idf_args = []

    def add_arguments(self, *, parser: argparse.ArgumentParser) -> None:
        parser.add_argument("package",
                            help="Specify ESP_IDF ROS2 package")
        parser.add_argument("idf_args", nargs="*",
                            help="Additional arguments will be passed to idf.py")

    def main(self, *, context: CommandContext):
        base_path = "/".join(os.environ.get("COLCON_PREFIX_PATH", "/").split("/")[:-1])
        base_path = base_path+"/" if len(base_path) > 0 else base_path
        cmd = ["idf.py", *context.args.idf_args]

        print(f"Running '{' '.join(cmd)}' for '{context.args.package}' package.")
        result = run(cmd, cwd=f'{base_path}src/{context.args.package}/{context.args.package}')
        
        return result.returncode
