"""Show the actual held-out skill curve, including regressions and the baseline."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent


if __name__ == '__main__':
    root = HERE / 'outputs/long-r2-hour'
    training = json.loads((root / 'training.json').read_text())
    history = json.loads((root / 'validation_history.json').read_text())
    initial = json.loads((root / 'validation_history/warm-initial.json').read_text())
    zero = json.loads((root / 'validation_history/zero-initial.json').read_text())
    xs = [0.] + [r['elapsed_seconds'] / 60 for r in history] + [training['learn_wall_seconds'] / 60]
    ys = [initial['total_successes_of_36']] + [r['total_successes_of_36'] for r in history] + [training['final_validation']['total_successes_of_36']]
    plt.rcParams['font.family'] = 'Microsoft YaHei'
    fig, axis = plt.subplots(figsize=(8, 4.5), layout='constrained')
    axis.plot(xs, ys, '-o', color='#1967a3', label='每次独立验证成绩')
    axis.axhline(zero['total_successes_of_36'], linestyle='--', color='#c27613', label='零修正程序基线')
    axis.axvline(9., color='#cccccc', linestyle=':')
    axis.axvline(24., color='#cccccc', linestyle=':')
    axis.set(xlabel='学习循环经过时间（分钟，含周期验证）', ylabel='抓取/抬升成功数（共36场）',
             ylim=(0, 36), title='训练更久，成绩也可能回落；选中权重单独保存')
    axis.set_yticks(range(0, 37, 6))
    axis.grid(alpha=.15)
    axis.legend(loc='lower left')
    fig.savefig(HERE / 'delivery/training_validation.png', dpi=150)
