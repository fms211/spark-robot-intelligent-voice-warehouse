#!/usr/bin/env python3
"""Write a disabled course calibration candidate from move2grasp coefficients."""
import argparse
from pathlib import Path
import yaml
from spark_course_mission.legacy_calibration import convert_legacy_calibration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='legacy thefile.txt')
    parser.add_argument('output', type=Path, help='new YAML file; must not already exist')
    args = parser.parse_args()
    try:
        candidate = convert_legacy_calibration(args.source.read_text())
        with args.output.open('x') as output:
            output.write('# Candidate only: validate current geometry and all field parameters.\n')
            yaml.safe_dump(candidate, output, default_flow_style=False, sort_keys=False)
    except (OSError, ValueError) as exc:
        parser.exit(2, 'Cannot create candidate: {}\n'.format(exc))
    print('Created disabled candidate: {}'.format(args.output))


if __name__ == '__main__':
    main()
