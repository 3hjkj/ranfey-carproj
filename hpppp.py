import os
import re

# 要处理的文本文件后缀
TEXT_FILE_EXTS = {".cpp", ".hpp", ".h", ".c", ".cc", ".cxx", ".py"}

# 文件路径匹配正则：例如 #include "some_package/msg/some_msg.hpp"
INCLUDE_REGEX = re.compile(r'#include\s*[<"]([\w_]+/msg/([\w_]+))\.h[>"]')


def camel_to_snake(name):
    # 转换 CamelCase 为 snake_case
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def process_file(filepath):
    changed = False
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        match = INCLUDE_REGEX.search(line)
        if match:
            full_path, msg_name = match.groups()
            new_path = f"{os.path.dirname(full_path)}/{camel_to_snake(msg_name)}.hpp"
            new_line = re.sub(rf"{re.escape(full_path)}\.h", new_path, line)
            print(
                f"[修改] {filepath}:\n  原始行: {line.strip()}\n  替换为: {new_line.strip()}"
            )
            new_lines.append(new_line)
            changed = True
        else:
            new_lines.append(line)

    if changed:
        with open(filepath, "w", encoding="utf-8") as f:
            f.writelines(new_lines)


def scan_project(root="."):
    for dirpath, _, filenames in os.walk(root):
        for filename in filenames:
            if any(filename.endswith(ext) for ext in TEXT_FILE_EXTS):
                process_file(os.path.join(dirpath, filename))


if __name__ == "__main__":
    scan_project()
