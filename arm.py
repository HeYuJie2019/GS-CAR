# 初使动作
def arm_initialize():
    S.write(bytes.fromhex('ff 01 b 14 0'))
    S.write(bytes.fromhex('ff 02 b c6 4'))
    S.write(bytes.fromhex('ff 01 a 14 0'))
    S.write(bytes.fromhex('ff 02 a 35 5'))
    S.write(bytes.fromhex('ff 01 9 14 0'))
    S.write(bytes.fromhex('ff 02 9 82 6'))
    S.write(bytes.fromhex('ff 01 8 14 0'))
    S.write(bytes.fromhex('ff 02 8 6c 5'))
# 看转盘动作
def arm_aim_turntable():
    S.write(bytes.fromhex('ff 1 b 14 0'))
    S.write(bytes.fromhex('ff 2 b 78 3'))
    S.write(bytes.fromhex('ff 1 a a 0'))
    S.write(bytes.fromhex('ff 2 a 62 4'))
    S.write(bytes.fromhex('ff 1 9 10 0'))
    S.write(bytes.fromhex('ff 2 9 60 7'))
    S.write(bytes.fromhex('ff 1 8 10 0'))
    S.write(bytes.fromhex('ff 2 8 2b 3'))
# 看绿靶动作
def arm_aim_bullseye():
    S.write(bytes.fromhex('ff 1 b 14 0'))
    S.write(bytes.fromhex('ff 2 b 78 3'))
    S.write(bytes.fromhex('ff 1 a a 0'))
    S.write(bytes.fromhex('ff 2 a c5 6'))
    S.write(bytes.fromhex('ff 1 9 10 0'))
    S.write(bytes.fromhex('ff 2 9 13 6'))
    S.write(bytes.fromhex('ff 1 8 10 0'))
    S.write(bytes.fromhex('ff 2 8 2b 3'))
# 抓取动作
def arm_grab():
    S.write(bytes.fromhex('ff 1 b 14 0'))
    S.write(bytes.fromhex('ff 2 b 13 6'))
# 放手动作
def arm_losse():
    S.write(bytes.fromhex('ff 1 b 14 0'))
    S.write(bytes.fromhex('ff 2 b 41 3'))

def 抓转盘():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 14 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 82 06'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 4b 06'))
    
    
def 放二号物料():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 c4 07'))
    S.write(bytes.fromhex('ff 01 09 0e 00'))
    S.write(bytes.fromhex('ff 02 09 3f 07'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a fd 04'))

def 放一号物料():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 c4 08'))
    S.write(bytes.fromhex('ff 01 09 0e 00'))
    S.write(bytes.fromhex('ff 02 09 60 07'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a e7 04'))

def 放三号物料():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 db 06'))
    S.write(bytes.fromhex('ff 01 09 0e 00'))
    S.write(bytes.fromhex('ff 02 09 60 07'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a e7 04'))

def 准备放三号物料台2():
    S.write(bytes.fromhex('ff 01 08 14 00'))
    S.write(bytes.fromhex('ff 02 08 db 06'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 29 07'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 78 04'))

def 准备放二号物料台2():
    S.write(bytes.fromhex('ff 01 08 14 00'))
    S.write(bytes.fromhex('ff 02 08 c4 07'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 29 07'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 78 04'))

def 准备放一号物料台2():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 c4 08'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 29 07'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 57 04'))

def 准备放一号物料台():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 c4 08'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 13 06'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 57 04'))

def 准备放三号物料台():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 c5 06'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 13 06'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 78 04'))

def 准备放二号物料台():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 c4 07'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 13 06'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 78 04'))

def 过渡():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 af 06'))
    S.write(bytes.fromhex('ff 01 09 0c 00'))
    S.write(bytes.fromhex('ff 02 09 6c 05'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a fd 04'))

def 准备抓一号靶0():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 58 02'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 13 06'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a 6c 05'))

def 准备抓一号靶():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 79 02'))
    S.write(bytes.fromhex('ff 01 09 0a 00'))
    S.write(bytes.fromhex('ff 02 09 dc 04'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 77 07'))

def 准备抓二号靶0():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 2b 03'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 fd 05'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 8e 05'))

def 二层准备三号0():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 14 04'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 dc 05'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a 8e 04'))

def 二层准备三号():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 14 04'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 af 05'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 77 06'))

def 二层准备二号0():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 36 03'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 a4 06'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a 8e 04'))

def 二层准备二号():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 2b 03'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 8d 06'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 6c 06'))

def 二层准备一号0():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 4c 02'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 34 06'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a 8e 04'))

def 二层准备一号():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 4c 02'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 dc 05'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 82 06'))

def 二层准备():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 35 05'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 dc 05'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a dc 05'))
    S.write(bytes.fromhex('ff 01 0b 0a 00'))
    S.write(bytes.fromhex('ff 02 0b fd 05'))

def 准备():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 4b 05'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 56 05'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a db 06'))
    S.write(bytes.fromhex('ff 01 0b 0a 00'))
    S.write(bytes.fromhex('ff 02 0b fd 05'))

def 准备抓三号靶0():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 fe 03'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 56 05'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a db 06'))

def 准备抓三号靶():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 14 04'))
    S.write(bytes.fromhex('ff 01 09 0a 00'))
    S.write(bytes.fromhex('ff 02 09 f2 04'))
    S.write(bytes.fromhex('ff 01 0a 0a 00'))
    S.write(bytes.fromhex('ff 02 0a 82 07'))

def 准备抓二号靶():
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 41 03'))
    S.write(bytes.fromhex('ff 01 09 08 00'))
    S.write(bytes.fromhex('ff 02 09 61 05'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a d0 07'))
   
def 正对():
    S.write(bytes.fromhex('ff 01 08 14 00'))
    S.write(bytes.fromhex('ff 02 08 41 03'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 82 06'))
    S.write(bytes.fromhex('ff 01 0a 10 00'))
    S.write(bytes.fromhex('ff 02 0a 35 05'))
    S.write(bytes.fromhex('ff 01 0b 0a 00'))
    S.write(bytes.fromhex('ff 02 0b c6 04'))

def 初始动作():
    S.write(bytes.fromhex('ff 01 08 0a 00'))
    S.write(bytes.fromhex('ff 02 08 6c 05'))
    S.write(bytes.fromhex('ff 01 09 0a 00'))
    S.write(bytes.fromhex('ff 02 09 82 06'))
    S.write(bytes.fromhex('ff 01 0a 0a 00'))
    S.write(bytes.fromhex('ff 02 0a 35 05'))
    S.write(bytes.fromhex('ff 01 0b 0a 00'))
    S.write(bytes.fromhex('ff 02 0b c6 04'))

def 抓():
    S.write(bytes.fromhex('ff 01 0b 1e 00'))
    S.write(bytes.fromhex('ff 02 0b f2 05'))

def 松():
    S.write(bytes.fromhex('ff 01 0b 1e 00'))
    S.write(bytes.fromhex('ff 02 0b 8e 04'))

def 看物料():
    S.write(bytes.fromhex('ff 01 08 14 00'))
    S.write(bytes.fromhex('ff 02 08 14 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 35 05'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a c6 04'))