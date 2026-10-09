"""Read-only UI readiness facts. Does not construct the hardware adapter."""
def readiness(store):
    status=store.status(); state=status.get('state') or {}; missing={}; trial_axes=[]
    for i in range(1,7):
        profile=state.get('drafts',{}).get(str(i)) or state.get('approved',{}).get(str(i)) or {}
        fields=profile.get('fields',{})
        if profile.get('source')=='user-provided-mit-initial-trial': trial_axes.append(i)
        missing[str(i)]=[name for name in ('v_max','a_max','jerk_max','error_max','torque_max','timeout') if fields.get(name,{}).get('value') is None]
    return dict(execution_state='LOCKED',executor_label='适配器待现场验收',hardware_allowed=False,
        adapter_implemented=True,auto_connect=False,configuration_revision=state.get('revision'),
        user_trial_axes=trial_axes,missing_constraints=missing,
        reasons=[
            '尚无当次现场许可：需确认人在设备旁、独立断电手段及承重关节支撑。',
            '先验收一个已支撑关节的小幅运动、取消、失联和停止；当前没有通过记录。',
            '试验速度、加速度、jerk、误差及转矩包络待逐项批准，不能用 PP/CSP 限速代替。',
            '实发前须退出其他 CAN 控制器并重新读取起点；网页配置批准不会解除此限制。',
        ],
        stop_policy='仅实现显式授权的已支撑轴撤力；未实现/批准承重轴保持策略',
        xyz_hardware_integrated=False,physical_acceptance=False,
        next_step='现场条件确认 → 单轴试验 → 再开放 XYZ 实机执行')
