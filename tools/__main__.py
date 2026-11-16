"""Command entry point for positioning tools."""

import argparse
from typing import Optional, Sequence


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Show the tool entry point until concrete experiment commands are added."""
    parser = argparse.ArgumentParser(
        prog="positioning",
        description="无人机定位工具入口。当前仅建立代码骨架，尚未实现实验命令。",
        epilog="算法和工作流放在 tools，独立公共函数放在 utils。",
    )
    parser.parse_args(argv)
    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
