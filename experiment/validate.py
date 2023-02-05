import subprocess
import platform
from pathlib import Path
import csv

project_root = Path(__file__).parent.parent
program = project_root / "cmake-build-relwithdebinfo" / "MAPF_Plan_Validator"
if platform.system() == "Windows":
    program = project_root / "cmake-build-relwithdebinfo" / "MAPF_Plan_Validator.exe"
else:
    program = project_root / "cmake-build-relwithdebinfo" / "MAPF_Plan_Validator"

result_dir = project_root / "result"
data_dir = project_root / "data"

def validate(file):
    p = subprocess.run([program, "--plan", str(file)], stdout=subprocess.PIPE)
    result = p.stdout.decode('utf-8').strip()
    return result


def main():
    with (data_dir / "validation.csv").open("w") as csvfile:
        for file in result_dir.glob("*.cbs"):
            result = validate(file)
            csvfile.write(f"{file.name},{result}\n")

if __name__ == '__main__':
    main()

