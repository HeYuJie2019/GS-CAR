def _init():
    global _global_dict
    acceleration = [0, 0, 0] #加速度
    angle_degree = [0, 0, 0] #欧拉角
    new_angle_degree = [0, 0, 0] #欧拉角
    img = 0
    _global_dict = {'jsd':acceleration}
    _global_dict = {'jd':angle_degree}
    _global_dict = {'frame':img}
    _global_dict = {'JD':new_angle_degree}

def set_value(key,value):
    _global_dict[key] = value

def get_value(key,defValue=None):
    try:
        return _global_dict[key]
    except KeyError:
        return defValue
