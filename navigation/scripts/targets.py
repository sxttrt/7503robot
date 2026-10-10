"""方案B: 目标点序列(只给目标点, 不给航点)。

顺序(规格要求): 起点 -> 桌前方二维码扫描点 -> 桌子正下方 -> 终点卸货点
                -> 下一个货物二维码扫描点 -> ... 循环 -> 终点正前方
ignore: 该目标点允许"进入"的货架(钻到它正下方时必须豁免它自身)。
"""
from scene import RACKS, DROP, END_SCAN, ORDER, SCAN_OFF, ROBOT_W, RACK_L

def build():
    T = []
    for index,n in enumerate(ORDER):
        x0, x1, y0, y1, sy, qr = RACKS[n]
        T.append(dict(name=f"scan_{n}",  xy=(x1+SCAN_OFF, sy), ignore=(),   qr=qr, rack=n, auto_release=index>0, note="货架前方二维码扫描点"))
        T.append(dict(name=f"under_{n}", xy=((x0+x1)/2, (y0+y1)/2),   ignore=(n,), qr=None,
                      rack=n, carry=True, note="桌子正下方(放行后=已举升, 车上带货架)"))
        T.append(dict(name=f"drop_{n}", xy=DROP[n], rack=n, ignore=(), qr=None, note="指定投放点"))
    T.append(dict(name="END", xy=END_SCAN, ignore=(), qr="END", note="终点正前方"))
    T.append(dict(name="START", xy=None, ignore=(), qr="START", note="起点(仅作序列头)"))
    return T[:-1]

def fit_ok():
    """车体能否钻入货架: 车宽 < 架口净宽。"""
    return ROBOT_W < RACK_L
