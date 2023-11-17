def _init():
    global _global_dict
    acceleration = [0, 0, 0] #加速度
    angle_degree = [0, 0, 0] #欧拉角
    img1 = None
    img2 = None
    speed_a = 0
    speed_b = 0
    speed_c = 0
    speed_d = 0
    target_speed_a = 0
    target_speed_b = 0
    target_speed_c = 0
    target_speed_d = 0
    model = 0
    frame_up_flag = 0
    _global_dict = {'jsd':acceleration}
    _global_dict = {'frame_up':img1}
    _global_dict = {'frame_down':img2}
    _global_dict = {'JD':angle_degree}
    _global_dict = {'motorA':speed_a}
    _global_dict = {'motorB':speed_b}
    _global_dict = {'motorC':speed_c}
    _global_dict = {'motorD':speed_d}
    _global_dict = {'motorA_':speed_a}
    _global_dict = {'motorB_':speed_b}
    _global_dict = {'motorC_':speed_c}
    _global_dict = {'motorD_':speed_d}
    _global_dict = {'targetA':target_speed_a}
    _global_dict = {'targetB':target_speed_b}
    _global_dict = {'targetC':target_speed_c}
    _global_dict = {'targetD':target_speed_d}
    _global_dict = {'model':model}
    _global_dict = {'frame_up_flag':frame_up_flag}

def set_value(key,value):
    _global_dict[key] = value

def get_value(key,defValue=None):
    try:
        return _global_dict[key]
    except KeyError:
        return defValue
