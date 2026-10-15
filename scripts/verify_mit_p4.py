#!/usr/bin/env python3
"""Reproduce P4 adapter + configuration tests with virtual SDK only. No CAN."""
import argparse
import importlib.util
import json
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=ROOT/'outputs/mit-p4/offline');args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True); suite=unittest.TestSuite()
    for name in ('test_mit_hardware_offline','test_config_protection'):
        spec=importlib.util.spec_from_file_location(name,ROOT/'tests'/f'{name}.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
    def cases(s):
        for item in s:
            if isinstance(item,unittest.TestSuite): yield from cases(item)
            else: yield item.id()
    names=list(cases(suite))
    with (args.output/'tests.txt').open('w') as stream: result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    bad={case.id():str(error) for case,error in result.errors+result.failures}
    report=dict(stage='P4-offline',passed=result.wasSuccessful(),tests_run=result.testsRun,cases=[dict(id=name,passed=name not in bad) for name in names],
        source='fake-SDK-and-virtual-clock',real_CAN_queries=0,real_motion_commands=0,physical_acceptance=False,
        limitations=['Not physics or real-time validation','Real multi-axis authorize is explicitly denied','No browser hardware execute endpoint',
        'Supported-release requires explicit onsite permission and mechanical support','A process crash or blocked SDK requires independent physical stop'])
    (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False));return 0 if result.wasSuccessful() else 1
if __name__=='__main__': raise SystemExit(main())
