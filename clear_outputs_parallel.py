import os
import subprocess
from concurrent.futures import ThreadPoolExecutor

def clear_ipynb_output(file_path):
    cmd = [
        "jupyter",
        "nbconvert",
        "--to",
        "notebook",
        "--ClearOutputPreprocessor.enabled=True",
        "--inplace",
        file_path,
    ]
    subprocess.run(cmd)

# 收集所有 .ipynb 文件
ipynb_files = []
for root, _, files in os.walk("."):
    for file in files:
        if file.endswith(".ipynb"):
            full_path = os.path.join(root, file)
            ipynb_files.append(full_path)

# 并行清除输出
with ThreadPoolExecutor() as executor:
    executor.map(clear_ipynb_output, ipynb_files)
