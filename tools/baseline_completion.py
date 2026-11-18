"""Finish comparisons and a report after explicitly named offline runs complete."""

import argparse
import json
from pathlib import Path
import time

from tools.baseline_report import BaselineReport
from tools.compare_lio import LioComparison


class BaselineCompletion:
    """Monitor local result files; never substitute partial trajectories for full runs."""

    def __init__(self, arguments):
        self.arguments = arguments

    def run(self):
        arguments = self.arguments
        pending = json.loads(arguments.pending.read_text())
        deadline = time.monotonic()+arguments.timeout_hours*3600
        while pending:
            if time.monotonic() > deadline:
                raise TimeoutError('Pending full replays exceeded the reporting deadline')
            for label, directory in list(pending.items()):
                replay = Path(directory)
                result_path = replay/'result.json'
                if not result_path.exists():
                    continue
                try:
                    result = json.loads(result_path.read_text())
                except json.JSONDecodeError:
                    # A runner may be replacing its final result at this instant.
                    continue
                if result['status'] == 'failed':
                    raise RuntimeError('{} replay failed: {}'.format(label, result.get('error')))
                if result['status'] != 'completed':
                    continue
                matching = []
                for path in arguments.reference.parent.glob('*/comparison.json'):
                    comparison = json.loads(path.read_text())
                    if comparison.get('candidate_label') == label and comparison.get('source_replay') == str(replay):
                        matching.append(path)
                if not matching:
                    LioComparison(arguments.reference, replay, label).run()
                del pending[label]
            if pending:
                time.sleep(15)
        BaselineReport(argparse.Namespace(comparison_root=arguments.reference.parent,
                                         output=arguments.output)).run()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--pending', type=Path, required=True,
                        help='JSON mapping method labels to full replay directories')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-hours', type=float, default=24)
    BaselineCompletion(parser.parse_args()).run()


if __name__ == '__main__':
    main()
