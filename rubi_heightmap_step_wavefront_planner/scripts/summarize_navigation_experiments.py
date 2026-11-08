#!/usr/bin/env python3
"""Summarize measured runs; keep failures and QC exclusions visible."""
import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    rows, groups, incomplete = [], defaultdict(list), []
    for path in sorted(args.directory.glob('*/events.jsonl')):
        result_path = path.parent/'result.json'
        if not result_path.exists():
            incomplete.append(str(path.parent))
            continue
        r = json.loads(result_path.read_text())
        p = r['parameters']
        initial = r.get('initial_plan') or {}
        row = dict(run_id=p['run_id'], scenario=p['scenario'], mode=p['cost_mode'],
                   outcome=r['outcome'], qc_pass=r['qc_pass'], qc_errors=';'.join(r['qc_errors']),
                   model_sha256=initial.get('model_sha256'),
                   polynomial_degree=len(initial.get('coefficients_descending_cm', []))-1,
                   height_weight=initial.get('height_weight'),
                   planned_length_m=initial.get('planned_length_m'),
                   tracking_length_m=initial.get('tracking_length_m'),
                   planned_ascent_m=initial.get('planned_cumulative_ascent_m'),
                   planned_descent_m=initial.get('planned_cumulative_descent_m'),
                   planner_total_cost=initial.get('total_cost'),
                   planning_ms_mean=r['planning_ms_mean'], accepted_replans=r['accepted_replans'])
        row.update(r.get('metrics') or {})
        rows.append(row)
        # Never pool different models or weights silently.
        key = (row['scenario'], row['mode'], row['model_sha256'],
               row['polynomial_degree'], row['height_weight'])
        groups[key].append(row)
    if not rows:
        print('No finalized runs. Incomplete run directories:', incomplete)
        return
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with (args.directory/'comparison_trials.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fields); writer.writeheader(); writer.writerows(rows)
    summary = []
    metrics = ['W_abs_J', 'CoT_abs', 'CoT_net_displacement', 'executed_length_xy_m',
               'planned_length_m', 'duration_s', 'P_abs_peak_W', 'planning_ms_mean', 'accepted_replans']
    for key, values in groups.items():
        valid_success = [r for r in values if r['qc_pass'] and r['outcome'] == 'success']
        row = dict(zip(['scenario','mode','model_sha256','polynomial_degree','height_weight'], key))
        row.update(n_finalized=len(values), n_operator_success=sum(r['outcome']=='success' for r in values),
                   n_qc_failed=sum(not r['qc_pass'] for r in values), n_success_qc_valid=len(valid_success))
        row['success_fraction_finalized'] = row['n_operator_success']/len(values)
        for metric in metrics:
            v = [r[metric] for r in valid_success if r.get(metric) is not None]
            row[metric+'_mean'] = statistics.mean(v) if v else None
            row[metric+'_SD'] = statistics.stdev(v) if len(v)>1 else None
        summary.append(row)
    with (args.directory/'comparison_summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, list(summary[0])); writer.writeheader(); writer.writerows(summary)
    (args.directory/'comparison_summary.json').write_text(json.dumps(dict(
        note='Energy summaries condition on success AND QC pass. Audit incomplete runs before reporting success rates.',
        incomplete_runs=incomplete, groups=summary), indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    if incomplete:
        print('INCOMPLETE RUNS (not in finalized denominator):', incomplete)


if __name__ == '__main__':
    main()
