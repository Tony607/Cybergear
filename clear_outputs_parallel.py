import os
import subprocess
from concurrent.futures import ThreadPoolExecutor
import re
import nbformat

def clean_execute_time(file_path):
    nb = nbformat.read(file_path, as_version=4)
    for cell in nb.cells:
        if "metadata" in cell:
            cell["metadata"] = {}
            
    nbformat.write(nb, file_path)
            
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
            clean_execute_time(full_path)

# 并行清除输出
with ThreadPoolExecutor() as executor:
    executor.map(clear_ipynb_output, ipynb_files)
