import shutil
from experiment.logger import logger
from experiment.utils import project_root
from tqdm import tqdm

# result_dir = project_root / "result"
result_dir = project_root / "test_result"


files = list(result_dir.iterdir())


# for file in tqdm(files):
#     if file.name.endswith("-eecbs.cbs") and file.name.startswith("discrete"):
#         arr = file.name.split("-")
#         if arr[-4] != "2":
#             arr[0] = "continuous"
#             arr.insert(-3, "2")
#             filename = "-".join(arr)
#             continuous_file = result_dir / filename
#             if not continuous_file.exists():
#                 shutil.copy(file, continuous_file)
#                 logger.info("copy {} -> {}", file.name, filename)

for file in tqdm(files):
    if file.name.endswith("-ccbs.cbs") and file.name.startswith("continuous"):
        arr = file.name.split("-")
        logger.info(file.name)

        # if arr[-4] != "2":
            # arr[0] = "continuous"
            # arr.insert(-3, "2")
            # filename = "-".join(arr)
            # continuous_file = result_dir / filename
            # if not continuous_file.exists():
            #     # shutil.copy(file, continuous_file)
            #     logger.info("copy {} -> {}", file.name, filename)

