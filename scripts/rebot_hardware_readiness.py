"""Read-only UI readiness facts. Does not construct the hardware adapter."""
def readiness(store):
    status=store.status(); state=status.get('state') or {}; missing={}; trial_axes=[]
    for i in range(1,7):
        profile=state.get('drafts',{}).get(str(i)) or state.get('approved',{}).get(str(i)) or {}
        fields=profile.get('fields',{})
        if profile.get('source')=='user-provided-mit-initial-trial': trial_axes.append(i)
        missing[str(i)]=[name for name in ('v_max','a_max','jerk_max','error_max','torque_max','timeout') if fields.get(name,{}).get('value') is None]
    return dict(execution_state='AWAITING_ONSITE_TASK',executor_label='Seeed SDK + P4 实机执行器',hardware_allowed=False,
        adapter_implemented=True,auto_connect=False,configuration_revision=state.get('revision'),
        user_trial_axes=trial_axes,missing_constraints=missing,
        reasons=[
            '每个任务由本机用户点击执行，并沿用上方一次现场确认。',
            '原生 CSP 已完成一次小幅六轴 XYZ 反馈到位；尚不代表全工作空间、J6 主动转动或夹爪验收。MIT 草稿 3/1/0 不作为 CSP 设备 PID。',
            '执行按既有临时试验包络检查，预演结果显示具体出力及时序阻挡。',
            '实发前须退出其他 CAN 控制器并重新读取起点；网页配置批准不会解除此限制。',
        ],
        stop_policy='当次任务完成、取消或异常后撤力；CSP 持续稳定及失能状态确认后恢复临时参数，不自动长期保持',
        xyz_hardware_integrated=True,physical_acceptance=False,
        next_step='实机起点预演 → 查看本任务出力/时序检查 → 现场确认 → 显式请求执行')
