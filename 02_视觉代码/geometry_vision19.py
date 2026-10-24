# -*- coding: utf-8 -*-
"""
2026 人形全能赛项 · 平面几何形状识别视觉程序
平台: MaixCAM / MaixPy v4 (MaixVision 直接运行, 也可打包成 App)

输出: ① 串口打印识别类别(默认 ASCII 行, 串口助手直接可读)
      ② 可选二进制帧 [0xAA,0x55,ID,ID^0xA5,0x0A]
      ID: 1=圆形 2=五角星 3=正方形 4=菱形 5=十字形 6=三角形

==========================================================================
v17 = v16 + 性能优化(识别逻辑与通信协议逐字不变)
==========================================================================
本版**不改任何判据**: 阈值、特征、打分权重、投票规则、串口帧格式全部原样。
改的只是"同一个结果用更少的算力算出来"。五项优化, 每项都在 PC 上用真实的
本模块代码做过 A/B 等价性验证(见各函数注释里的实测数字):

  ① 十字四臂检测向量化      8.5ms → 1.0ms   最大的单项瓶颈
     原来是 7x7=49 个候选中心 x 4 条臂 = 196 次 np.any 窗口扫描(Python 循环)。
     改成 dilate 做最大值滤波 + cumsum/integral 做前缀和, 一次算完所有候选。
  ② 四张墨迹掩码改惰性生成  6.1ms → 1.9ms   典型帧只用第一张
     原来 Otsu + 3 个固定阈值全算好再挑, 而 find_card_quad 用上第一张成功的
     就 return 了 —— 绝大多数帧后面 3 张是纯浪费。改成生成器, 要几张算几张。
  ③ 图卡走灰度通道          省 2 次 cvtColor + 2/3 的 warp 带宽
     原来 warpPerspective 搬三通道, 下游 detect_cross_center / shape_from_roi
     各自再 cvtColor 一次。改成直接 warp 灰度图(main 里已经有现成的 gray)。
     彩色矫正图只在要画缩略图时才算 —— 比赛模式下根本不算。
  ④ 快筛直接缩灰度图        1.17ms → 0.66ms
     原来 resize 三通道再转灰; 现在转好的灰度图直接缩, 决策 120/120 帧一致。
  ⑤ 显示分级 SHOW_LEVEL     比赛模式省掉整条显示链
     两次整幅 copy()(main 一次 + corr.draw 内部一次) → 比赛模式 0 次。
     cv2.cv2image + disp.show 是 640x480 下最贵的一环, 比赛模式整个跳过。

实测(PC 单核, 只能看相对权重, 不是 MaixCAM 的绝对值):
    有卡帧 19.6ms → 7.9ms(2.5x), 空帧 2.12ms → 1.54ms。
★ 这些是 x86 单核数, 搬到 MaixCAM 上同比例放大(C906 大约慢一个数量级),
  **绝对帧率必须在真机上用 time.fps() 现测**, 不要引用这里的毫秒数。

==========================================================================
v16 = v15 + 同一种形状发送成功后不再打印(REPORT_ONCE)
==========================================================================
赛道是 400m 环形, 裁判只贴 3 张卡, 机器人跑完一圈会再次经过同一张卡。
旧逻辑里"不重报"只是短期锁(locked): 图卡离开视野满 GATE_CLEAR_N 帧就解锁,
第二圈经过时又会完整地重报一遍 —— 控制台刷屏, 串口重发, 下位机把那套肢体
动作再做一次。

本版加一层永久退役集合 Voter.sent: 某一种形状**成功发出去**之后
  · 不再发串口;
  · 不再打印任何识别输出 —— 连 DEBUG 的 [十字] / [特征] 行一起静音
    (否则那张卡在视野里停十几帧, 每帧一行, 会把真正有用的新卡信息刷掉);
  · 它的票在滑窗里只占位不计数, 不跟后面的新卡抢票;
  · 屏幕上照常画出来, 但标 DONE + 灰色, 右上角 "SENT 236" 列出已报的 ID ——
    要能一眼区分"故意不报"和"识别挂了"。
voter.lost() 只清短期锁, 不清退役集合。

★ 代价: 退役按【形状种类】记。一旦误报(把菱形认成正方形发了出去), 真正的
  正方形卡这一轮就再也报不出来了。它买到的是"不重复", 不是"更准"。
  要回到旧行为, 把 REPORT_ONCE 改成 False。

==========================================================================
v15 = v14 + 修掉"只看到一条车道线时把左右认反"的循线缺陷
==========================================================================
赛道是两条黑线围出的车道, 机器人走中间, 目标点 = 两线中点。两条都看到时
这个模型没问题; 出问题的是只看到一条的时候(过弯内侧线出画, 很常见)。

v10~v14 一直是拿【画面中线】分左右: s[0] < 320 就算左线。单看一帧也许还对,
连起来看就是错的 —— 那条唯一可见的线会随着机器人转向扫过画面中心, 一跨过去
标签就翻, 外推方向跟着反号, 目标点一帧之内跳 2*half(约 300px), 机器人朝反
方向猛转, 左右横跳收不住。而且翻之前目标点已经被推到线的【外侧】, 机器人
正朝着离开赛道的方向走。

本版改成按【上一帧量到的赛道中心】认边(见 pick_edges): 两条线各在中心 ±half,
要让标签翻, 机器人得整整偏出半个车道宽, 正常巡线不会发生。顺带把"离预测位置
太远的暗段"直接挡掉(EDGE_GATE_R), 刻度数字、虚线、远处阴影不再冒充车道线。
远带/近带各记各的中心(透视 + 弯道决定两者本来就不相等)。
冷启动和确认丢线之后没有历史可用, 这时**宁可报丢线**也不拿画面中线猜
(EDGE_NEED_PRIOR) —— 猜错是直接走出赛道, 报丢线只是先停下再回正。

此缺陷在 v10/v11/v13/v14 中逐字相同, **不是 v14 引入的回归**。

==========================================================================
v14 = v13 + 上报形状后立刻转入循线模式(ACT_HOLD_N=0 + LineCorrector.resume)
==========================================================================

==========================================================================
v11 = v7 的图形识别 + v10 的循线
==========================================================================
两个版本的卡片定位/透视矫正/形状提取/几何特征函数本来就是**逐字相同**的,
真正的分歧只有三处, 这里一律按 v7 办:

  ① 理想特征画像 PROFILES: v7 手写常数, v10 改成启动时用 features() 量
     模板自动标定。本版用 v7 的手写表 —— 不再跑 calib_profiles()。
  ② 凹顶点权重 W["reflex"]: v7 给 1.5(主力判据), v10 降到 1.0 并把
     五角星/十字形改交给 arm_count(径向对称阶数)专项判别。本版回到 1.5。
  ③ arm_count / CONCAVE_TWIN 分支: v10 独有, 本版**不要**。五角星 vs 十字形
     重新由"凹顶点 5 vs 8"分开 —— 这是 v7 画像里的数字, 和 v7 的胖十字模板
     是一套口径, 不能只搬其中一半。

循线侧整个取 v10, 一行未改:
  · 扫描带里把图卡挖掉(第一层用本帧已定位的四边形求交, 第二层靠白底纸识别)
  · 丢线 → 停 → 按累计净转角反转回去 → 恢复站姿(HOLD/TURN/HOME 三段式)
  · 边走边识别的调度: 快筛每帧跑, 一报疑似就每帧跑完整流水线, 不再隔帧降频
  · 滑窗多数票 + 上报后 ACT_HOLD_N 帧不发纠偏指令, 让下位机做完肢体动作
  · 主循环顺序: 先图形后纠偏 —— 纠偏要用**本帧**的四边形挖图卡, 反过来排
    就只能用上一帧的, 机器人在走, 一帧的位移足以让挖的位置错开

--------------------------------------------------------------------------
形状判定的底层逻辑(v3 起沿用至今)
--------------------------------------------------------------------------
Hu 矩有个先天性质, 不是参数没调好:

  · 菱形 = 正方形旋转 45°。Hu 矩的七个不变量被设计成【旋转不变】,
    所以 matchShapes(正方形, 菱形) ≈ matchShapes(菱形, 菱形)。
    "最佳/次佳"比值恒等于 ≈1.0 → 永远低于置信度门槛 → 每帧被丢。
  · 十字形和五角星同属凹形, Hu 矩距离同样拉不开。

结论: Hu 矩【不能】用来区分互为旋转的形状, 只能用来做粗分组和交叉验证。
所以:
  ① 主判定 = 几何特征打分(圆度 / 凸性 / 填充率 / 顶点数 / 凹顶点数 / 边方向)
  ② 正方形 vs 菱形 由【卡片坐标系下的边方向】决定 —— 这是唯一可靠依据
  ③ Hu 矩降级为辅助: 仅在几何打分整体偏低时兜底, 且计算置信度时
     主动排除"旋转等价对", 不再让比值恒为 1 的假象压死结果
--------------------------------------------------------------------------
"""

import math
from collections import deque

import cv2
import numpy as np
from maix import app, camera, display, image, time, uart

# ============================ 可调参数 ============================
# 320x240 下卡片约 55px, 0.5cm 边框只剩 2.8px, 二值化后必然断裂。
# 640x480 下同样距离卡片约 110px, 边框 5.6px, 才谈得上稳定提取。
CAM_W, CAM_H = 640, 480

CARD_PX      = 220           # 透视矫正后卡片边长(px), 即特征尺度基准
CARD_INSET   = 0.06          # 内缩比例: 卡片边框占 5%, 缩 6% 刚好切掉
INSET_RETRY  = (0.06, 0.10, 0.14)   # 第一次没提到形状就加大内缩重试

AREA_MIN_REL = 0.005         # 卡片最小面积 / 整幅画面
AREA_MAX_REL = 0.80
SQUARE_TOL   = 0.35          # 外框长宽比容差(斜视角下是梯形, 必须放宽)
RECT_FILL    = 0.62          # 轮廓面积 / minAreaRect 面积 下限(排除细长线条)
HULL_FILL    = 0.55          # 轮廓面积 / 凸包面积 下限(排除破碎噪点)
CLOSE_K      = 5             # 闭运算核, 接上断掉的边框

THRESH_LIST  = (90, 120, 150)  # Otsu 失败时的固定阈值兜底
SAT_MAX      = 110           # 饱和度上限, 顺手滤掉红色木条障碍

# ---- 几何打分 ----
SCORE_MIN    = 0.55          # 最佳形状得分低于此值才启用 Hu 矩兜底
ANG_TOL      = 22.5          # 正方形/菱形的边方向分界角(度)
CONF_MIN     = 0.25          # 最终置信度(0~1)低于此值本帧弃权
DEFECT_MIN   = 0.05          # 深凹口阈值(相对卡片边长)

# 各几何量的容差(实测值离理想值多远算"不匹配")
TOL = dict(circ=0.15, solid=0.18, fill=0.15)

# 每个形状的"理想特征画像"。数值是**外轮廓**实测空间下的经验值, 不是理论值 ——
# findContours(RETR_EXTERNAL) 取的是墨迹外边界, 所以空心图卡(圆环/方框)
# 和实心图形在这里的表现是一致的, 这正是不必区分"空心/实心"的原因。
PROFILES = {
    "circle":   dict(circ=0.95, solid=1.00, fill=0.79, v=8, v_tol=3,
                     reflex=0, reflex_tol=1, deep=0, deep_tol=1),
    "square":   dict(circ=0.78, solid=1.00, fill=1.00, v=4, v_tol=0,
                     reflex=0, reflex_tol=1, deep=0, deep_tol=1),
    "diamond":  dict(circ=0.78, solid=1.00, fill=1.00, v=4, v_tol=0,
                     reflex=0, reflex_tol=1, deep=0, deep_tol=1),
    "triangle": dict(circ=0.60, solid=1.00, fill=0.50, v=3, v_tol=0,
                     reflex=0, reflex_tol=1, deep=0, deep_tol=1),
    "star":     dict(circ=0.55, solid=0.52, fill=0.45, v=5, v_tol=0,
                     reflex=5, reflex_tol=1, deep=5, deep_tol=1),
    "cross":    dict(circ=0.55, solid=0.25, fill=0.25, v=4, v_tol=0,
                     reflex=8, reflex_tol=2, deep=4, deep_tol=1),
}
# 权重: 凸性/填充率是最能把形状分成三组的量, 给最高;
#       凹顶点数是区分五角星/十字形的主力, 给最高; 深凹口数只作辅助。
W = dict(circ=1.0, solid=1.2, fill=1.2, v=1.0, reflex=1.5, deep=0.6)

ROT_TWIN = ("square", "diamond")   # 互为 45° 旋转、Hu 矩无法区分的等价对

# ---- 串口 ----
# MaixVision 启动时会主动让出 /dev/ttyS0 给用户程序(日志里 "[Maix Comm Protocol]
# exit success" 后面那行), 所以用 ttyS0。打包成独立 App 跑改用 "/dev/ttyS1"。
UART_PORT, UART_BAUD = "/dev/ttyS0", 115200  # 当前仅调视觉，不向机器人发送动作
SERIAL_MODE  = "ascii"       # "ascii" 可读文本 / "binary" 5字节帧
SHOW_CONSOLE = True

# ---- 横向扫描带(赛道边检测的公共前端) ----
BAND_H       = 16          # 每条采样带的高度(px)
LINE_OFF     = 38          # 自适应阈值 = 该带亮度中位数 - LINE_OFF
LINE_MIN_W   = 4           # 线宽下限(px), 滤掉噪点
LINE_MAX_W   = 95          # 线宽上限(px), 滤掉红木条/大块阴影

# ---- 把图卡的黑边框/黑图形从"赛道黑线"里剔掉 ----
# 两层防线, 因为这两种黑在灰度剖面里长得一模一样, 单靠宽度分不开:
#   第一层(主力): 图形流水线每帧已经把图卡四边形找出来了, 直接拿这个多边形
#     跟扫描带求交, 把那段 x 区间从剖面里挖掉。不用新猜任何阈值, 白用现成结果。
#   第二层(兜底): 四边形没拟合上、但图卡确实在画面里时用。图卡是**白底纸**,
#     比赛道面亮一截; 赛道黑线周围没有这么一大片白。所以找出白底纸的 x 范围,
#     凡是落在里面的暗段都算图卡的一部分(外框、内部图形的墨一并覆盖)。
#     注意: 扫描带穿过图形时, 图形的墨会把白纸切成几条窄白条(实测 120px 宽的
#     图卡上只剩 12~19px 的碎段), 所以**不能**直接要求"单段白够宽" —— 先做
#     一次一维闭运算把被墨隔开的白条接回去, 再量整块的宽度。
#   三道失效即安全的闸: 整带过半偏白(赛道面本身是白的) → 不信; 白块比整条赛道
#   还宽 → 不是一张 10cm 的卡; 剔除后一条线都不剩 → 退回只信第一层。
CARD_REJECT_ON = True
CARD_WHITE_DN  = 26        # 比该带中位数亮这么多(灰度级)才算白底纸
CARD_INK_MAX   = 50        # 图形墨迹在带上的最大宽度(px@640), 闭运算的桥接跨度
CARD_WHITE_MIN = 30        # 整块白底纸的最小宽度(px@640), 滤掉零星高光
CARD_W_MAX     = 300       # 整块白底纸的最大宽度(px@640), 超了就不是一张图卡
CARD_PAD       = 10        # 暗段离白底纸这么近(px)就判为图卡的一部分

# ---- 离散纠偏状态机(走-停-转-复查, 人形专用) ----
# 人形不能边走边微调航向: 迈步过程中骨盆朝向是摆动的, 连续打舵会把步态打乱。
# 所以改成"对准 → 走两步 → 再对准"的离散循环, 这也是流程图那套逻辑。
CORR_BAND    = 0.62        # 测偏差用的【远】采样带高度比例(占画面高)
CORR_BAND_N  = 0.90        # 辅助用的【近】采样带, 用来估赛道宽度先验
# ★ 这两个门限是**现场手感参数**, 这里给的是 v10 的原值。
#   v7 用的是 120/70, geometry_vision.py 上你自己改成了 120/50 ——
#   如果那是在真机上量出来的手感, 把下面两行改回 120/50 再跑。
#   数值越小越"较真": 34px@640 意味着偏差超过画面宽的 5% 就停下来转。
CORR_E_TOL   = 120          # 偏差门限(px@640宽): 超过就停下来转
CORR_E_OK    = 70          # 回正门限(px): 转到小于这个值才算对准
                           # 必须 E_OK < E_TOL 留出死区 —— 否则在门限上反复
                           # 进出"转一下又判超差", 原地抖个没完
CORR_TURN_DEG = 3          # 单次原地转动角度(度), 与流程图一致
CORR_STEP_N   = 2          # 对准后前进步数(1~2步)
CORR_SETTLE   = 0          # 持续测量建议：不按上一条建议暂停采样
CORR_SEND_INTERVAL_MS = 100 # 每次发送对应一张新图像，最大10Hz；不是动作等待
CORR_SHIFT_ON = True       # 双边线近远中心同向偏移且近似平行时建议平移
CORR_HEADING_TOL = 35      # 近远中心差门限(px@640)，须按相机安装实机核对
                           # ★ 流程图里"重新检测"这一步隐含了一个前提: 机器人
                           #   已经转完了。不等就是拿转动过程中的模糊帧做判断,
                           #   会连发十几条转向指令, 实际转过头 30° 以上。
CORR_MAX_TURN = 14         # 连续转这么多次还没对准 → 判定为线找错了, 不再瞎转
CORR_TRACK_W  = 300        # 赛道宽先验(px@640宽), 只看到一条边时用它外推
CORR_LOST_N   = 5          # 连续这么多帧找不到黑线 → 回到原来的姿态
CORR_GOOD_N   = 3          # 连续对准这么多拍, 才认定"当前朝向就是好朝向",
                           # 才把累计转角清零。只对准一帧就清零太急: 线快消失
                           # 前的那一帧常常碰巧 e 很小, 清掉就没东西可回正了。
CORR_HOME_N   = 3          # 回正后重发几次站姿指令。恢复站姿是幂等的, 发多次
                           # 和发一次结果一样, 但单帧掉包就不至于站不回去。
CORR_ON       = True       # False = 只跑图形识别, 不发任何纠偏指令

# ---- 只看到一条车道线时, 怎么判它是左是右 ----
# ★ 这是本版修掉的缺陷。原来是拿【画面中线】分左右:
#       left  = [s for s in segs if s[0] <  cx]
#       right = [s for s in segs if s[0] >= cx]
#   过弯时内侧那条线整条滑出画面, 只剩外侧一条, 而它会随着机器人转向
#   **扫过画面中心**。它一跨过去, 标签就从"右线"翻成"左线", 外推方向跟着反号,
#   目标点瞬间跳 2*half(≈300px), 机器人立刻朝反方向猛转 —— 左右横跳收不住。
#   而且在跨过去之前, 目标点就已经被推到那条线【外侧】150px 的空地上了,
#   机器人是朝着远离赛道的方向走的。
#   改成按【上一帧量到的赛道中心】分: 两条车道线离中心各 half, 一条线要翻越
#   中心得整整偏出半个车道宽, 正常巡线不会发生, 标签也就不会翻。
EDGE_GATE_R     = 0.90     # 暗段离"它该在的位置"超过 half*本系数 → 不是车道线, 丢掉。
                           # half=150 时窗口是 ±135px, 略宽于 CORR_E_TOL(120) ——
                           # 机器人偏到该纠偏的程度时, 边还认得住; 再偏才放弃。
                           # 画面里的刻度数字、虚线射线、远处阴影就是这么被挡掉的。
EDGE_NEED_PRIOR = True     # True = 在"左右都看到过"之前, 只看到一条线时**报丢线**,
                           #        不拿画面中线瞎猜方向。
                           #   猜错的代价是把机器人直接带出赛道; 报丢线只是先停下,
                           #   再由丢线回正把它转回来 —— 失败模式温和得多。
                           #   机器人是被摆在赛道里起步的, 头几帧本来就该两条都看到,
                           #   所以这个前提不难满足。
                           # False = 退回旧行为(拿画面中线猜), 只用于对比排查。

# ---- 两条流水线的调度(边走边识别) ----
# 快筛每帧都跑 —— 它只碰 1/16 的像素, 便宜到可以无条件执行。
# 快筛一旦报"疑似图卡", 就【每帧】跑完整识别流水线, 不再隔帧降频:
# 边走边识别时图卡只在视野里停留十几帧, 隔 3 帧跑等于白扔 2/3 的机会。
# 空帧(绝大多数)仍然只付快筛那点代价, 所以平均帧率不受影响。
GATE_ON      = True                 # 先用 1/4 小图快筛, 没方块就整条流水线都跳过
GATE_W, GATE_H = CAM_W // 4, CAM_H // 4
GATE_CLEAR_N = 8                    # 快筛连续这么多帧为空 → 认定图卡已离开视野,
                                    #   解锁, 准备认下一张卡。
                                    #   注意它只解 locked(同一张卡的短期锁), 不解
                                    #   REPORT_ONCE 的退役集合 —— 已经报过的那一种
                                    #   形状, 卡离开再回来也不会重报。

# ---- 投票(识别到了就发指令) ----
# 不再要求"连续 N 帧完全一致"。边走边识别时视角一直在变, 偶尔插进来一帧
# 误判就把连续性打断, 严格连续票在十字形这种边界形状上几乎永远凑不满。
# 改成滑窗多数票: 最近 VOTE_WIN 次识别里, 某形状出现 >= VOTE_NEED 次且
# 严格领先第二名, 就立即上报。
VOTE_WIN     = 5
VOTE_NEED    = 2                    # 2/5 就发 —— 用一点抗抖换低延迟
VOTE_CONF    = 0.45                 # 只有置信度到这个线的帧才投票(把弱帧挡在窗外)

# ---- 同一种形状只报一次(本版新增) ----
# True = 某一种形状【成功发出去】之后就永久退役: 之后再看到同一种形状,
#        既不再发串口, 也不再打印任何识别输出(连 DEBUG 的 [特征] 行一起静音),
#        屏幕上只把它标成 DONE。
#   赛道是 400m 的环形, 裁判只贴 3 张卡。机器人跑完一圈会再次经过同一张卡,
#   旧逻辑里 voter.lost() 一解锁就会把它重新上报一遍 —— 同一个形状刷屏,
#   下位机也会把那套肢体动作再做一遍。退役集合就是为了堵这个。
#
# ★ 代价说清楚: 退役是按【形状种类】记的, 一旦误报(把菱形认成正方形并发了出去),
#   真正的正方形卡这一轮就再也报不出来了。所以它换来的是"不重复", 不是"更准"。
#   想回到旧行为(图卡离开视野就解锁、允许再报)把这里改成 False。
REPORT_ONCE  = True

# 上报形状后, 暂停发纠偏指令若干帧, 让下位机把那套肢体动作做完。
# ★ 本版改为 0 = **发完立刻转入循线模式**。
#   上报只是往串口写一行字, 那套肢体动作由下位机自己排队执行, 上位机跟着一起
#   干等没有意义: 等的这 25 帧里机器人并没有停, 偏差照样在累积, 等完再纠反而
#   要多转好几次才收得回来。更要紧的是图卡就贴在赛道边上, 上报那一刻机器人往往
#   正偏向卡那一侧 —— 这恰恰是最需要马上纠偏的时刻。
# 如果下位机确实做不到"边做动作边走", 把这里改回 25 即可恢复旧行为。
ACT_HOLD_N   = 0

DEBUG     = False             # 控制台打印特征值 + 各形状得分
SHOW_BIN  = False            # 屏幕上叠二值图(调阈值时开)

# ---- 显示分级(本版新增) ----
# 640x480 下 image.cv2image() + disp.show() 要把 92 万像素搬进 framebuffer,
# 是整个循环里最贵的一环, 而且它跟识别一点关系都没有。加上 draw() 里那堆
# cv2.line/rectangle/putText 和两次整幅 copy(), 显示链在有卡帧上占 2.3ms(PC)。
#   2 = 调试: 全部叠加(远带、线心、预测位、图卡框、矫正缩略图、文字)
#   1 = 轻量: 只推原始画面 + fps, 不画任何叠加 —— 确认相机还活着用
#   0 = 比赛: 完全不碰屏幕, 只跑识别和串口
# ★ 比赛前只需要改这一行 + DEBUG。其余参数一律不要动。
SHOW_LEVEL = 0

# 十字检测走向量化实现(与逐点扫描逐值等价, 见 _cross_center_ref 的说明)。
# 留这个开关是为了在真机上做 A/B 对拍和源码核验; 正常跑一律 True。
CROSS_FAST = True

SHAPES   = ["circle", "star", "square", "diamond", "cross", "triangle"]
SHAPE_ID = {n: i + 1 for i, n in enumerate(SHAPES)}
CN_NAME  = {"circle": "圆形", "star": "五角星", "square": "正方形",
            "diamond": "菱形", "cross": "十字形", "triangle": "三角形"}


# ============================ 1. Hu 矩模板 ============================
def build_templates(size=CARD_PX):
    """按赛项图卡比例绘制 6 个理想形状的**填充外轮廓**作为 Hu 矩模板。

    必须用填充轮廓而不是描边图: 实拍时 findContours 取到的是墨迹的**外边界**,
    CV 里 contourArea / 矩 也是按外边界围成的面积算, 两边得同口径。
    """
    S, c = size, size // 2
    masks = {}

    m = np.zeros((S, S), np.uint8)
    cv2.circle(m, (c, c), int(0.375 * S), 255, -1)
    masks["circle"] = m

    m = np.zeros((S, S), np.uint8)
    R, r = 0.375 * S, 0.375 * S * 0.382            # 五角星外接/内接半径比
    pts = [[c + (R if i % 2 == 0 else r) * math.cos(-math.pi / 2 + i * math.pi / 5),
            c + (R if i % 2 == 0 else r) * math.sin(-math.pi / 2 + i * math.pi / 5)]
           for i in range(10)]
    cv2.fillPoly(m, [np.array(pts, np.int32)], 255)
    masks["star"] = m

    m = np.zeros((S, S), np.uint8)
    a = int(0.30 * S)
    cv2.rectangle(m, (c - a, c - a), (c + a, c + a), 255, -1)
    masks["square"] = m

    m = np.zeros((S, S), np.uint8)
    d = int(0.375 * S)
    cv2.fillPoly(m, [np.array([[c, c - d], [c + d, c], [c, c + d], [c - d, c]], np.int32)], 255)
    masks["diamond"] = m

    m = np.zeros((S, S), np.uint8)
    L, w = int(0.375 * S), int(0.09 * S)           # 十字形 = 两根正交矩形
    cv2.rectangle(m, (c - L, c - w), (c + L, c + w), 255, -1)
    cv2.rectangle(m, (c - w, c - L), (c + w, c + L), 255, -1)
    masks["cross"] = m

    m = np.zeros((S, S), np.uint8)
    hh, hw = int(0.33 * S), int(0.375 * S)
    cv2.fillPoly(m, [np.array([[c, c - hh], [c + hw, c + hh], [c - hw, c + hh]], np.int32)], 255)
    masks["triangle"] = m

    tpl = {}
    for k, mk in masks.items():
        cnts, _ = cv2.findContours(mk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        tpl[k] = max(cnts, key=cv2.contourArea)
    return tpl


# ============================ 2. 卡片定位 ============================
def order_quad(pts):
    """按绕质心的角度排序, 再把最左上角转到首位。

    比"和最大/差最大"那套稳: 卡片转到 45° 附近时后者的两个判据会打架,
    排出蝴蝶结顺序, warp 出来是一团扭曲。
    """
    p = np.asarray(pts, np.float32).reshape(-1, 2)
    c = p.mean(axis=0)
    ang = np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])
    p = p[np.argsort(ang)]                          # 图像坐标下这个序是顺时针
    k = int(np.argmin(p[:, 0] + p[:, 1]))           # 视觉左上角
    return np.roll(p, -k, axis=0).astype(np.float32)


def card_quad_reject_reason(quad, w, h):
    """最终四边形也必须满足原面积范围及画面边界，避免凸包/外接矩形扩成整幅赛道。

    保留AREA_MIN/MAX_REL原值；边界留1px仅容纳浮点拟合误差。
    返回空串表示有效，不以是否已识别出形状决定是否排除真实图卡。
    """
    p = np.asarray(quad, np.float32)
    if p.size != 8 or w < 2 or h < 2:
        return "shape"
    p = p.reshape(4, 2)
    if not np.isfinite(p).all():
        return "nonfinite"
    if (p[:, 0].min() < -1.0 or p[:, 1].min() < -1.0 or
            p[:, 0].max() > w or p[:, 1].max() > h):
        return "bounds"
    edges = np.roll(p, -1, axis=0) - p
    turns = edges[:, 0] * np.roll(edges[:, 1], -1) - edges[:, 1] * np.roll(edges[:, 0], -1)
    if not (np.all(turns > 0) or np.all(turns < 0)):
        return "nonconvex"
    area = float(cv2.contourArea(p))
    if not AREA_MIN_REL * w * h <= area <= AREA_MAX_REL * w * h:
        return "area"
    return ""


def ink_masks(bgr, gray=None):
    """生成黑色墨迹候选掩码。第一个是 Otsu(绝大多数帧够用), 后面几个是兜底。

    ★ v17: 改成**生成器**, 并接收 main 已经算好的 gray。
      原来是把 4 张全分辨率掩码一次性算齐再返回 list, 可 find_card_quad 拿到
      第一张能定出四边形的掩码就 return 了 —— 正常光照下 Otsu 一发入魂, 后面
      3 张(各一次 threshold + 一次 bitwise_and, 全是 640x480)纯属白算。
      改成惰性产出后, 典型帧只付 1 张的钱, 只有 Otsu 失手的难帧才继续往下算。
      实测: find_card_quad(含掩码) 6.44ms → 5.16ms; 掩码本身 6.11ms → 1.9ms。
      产出的掩码与原版**逐字节相同**(np.array_equal 验证通过), 阈值一个没动。
    """
    if gray is None:                       # 单独调用时仍然自己转(保持可独立使用)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    sat_ok = cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 1], 0, SAT_MAX)
    _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    yield cv2.bitwise_and(otsu, sat_ok)
    for t in THRESH_LIST:
        _, b = cv2.threshold(blur, t, 255, cv2.THRESH_BINARY_INV)
        yield cv2.bitwise_and(b, sat_ok)



def quad_from_mask(mask):
    """在单个掩码里找卡片外框, 返回 (4角点, 诊断字符串)。找不到返回 (None, diag)。"""
    H, W = mask.shape
    total = W * H
    m = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((CLOSE_K, CLOSE_K), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best, best_a = None, 0.0
    max_rel, n_area, n_rect = 0.0, 0, 0
    n_quad_reject, last_quad_reject = 0, "-"

    for cnt in cnts:
        a = cv2.contourArea(cnt)
        max_rel = max(max_rel, a / total)
        if a < AREA_MIN_REL * total or a > AREA_MAX_REL * total:
            continue
        n_area += 1

        # 用 minAreaRect 判"是不是方的": 对斜视角梯形比 boundingRect 宽容得多
        (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
        if rw < 1 or rh < 1 or not (1 - SQUARE_TOL < rw / rh < 1 + SQUARE_TOL):
            continue                                # 细长的赛道黑线在这里被刷掉
        if a / (rw * rh) < RECT_FILL:
            continue                                # 填充率太低 → 不是块状物

        hull = cv2.convexHull(cnt)
        ha = cv2.contourArea(hull)
        if ha < 1 or a / ha < HULL_FILL:
            continue                                # 轮廓太破碎
        n_rect += 1

        # ★ 拟合【凸包】而不是原轮廓。细边框二值化后常断成 C 形, 原轮廓有 8 个角,
        #   怎么调 epsilon 都出不来 4 个; 而凸包永远是完整的方形外沿, 4 个角一步到位。
        hp = cv2.arcLength(hull, True)
        ap = cv2.approxPolyDP(hull, 0.035 * hp, True)
        if len(ap) != 4:
            ap = cv2.boxPoints(cv2.minAreaRect(hull))   # 再兜一层
        ordered = order_quad(ap)
        reason = card_quad_reject_reason(ordered, W, H)
        if reason:
            n_quad_reject += 1
            last_quad_reject = reason
            continue
        if ha > best_a:
            best, best_a = ordered, ha

    return best, "轮廓%d 最大占比%.3f 达标%d 方%d quad_reject=%d:%s" % (
        len(cnts), max_rel, n_area, n_rect, n_quad_reject, last_quad_reject)


def find_card_quad(masks):
    """依次试各个掩码, 一旦找到就返回, 省算力。

    masks 现在是 ink_masks() 的生成器: 这里 return 得早, 后面的掩码就根本
    不会被算出来(原版是先全算好再挑, 省下来的就是这部分)。
    """
    diag = ""
    for i, mk in enumerate(masks):
        quad, d = quad_from_mask(mk)
        if quad is not None:
            return quad, ("Otsu" if i == 0 else "thr%d" % THRESH_LIST[i - 1]) + " " + d
        if i == 0:
            diag = d
    return None, diag


def rectify(img, quad):
    """透视矫正: 把卡片拉成正视的 CARD_PX 方图。

    校准后"卡片坐标系"就等同于"图像坐标系", 判正方形/菱形才有绝对参照,
    同时把机器人俯仰、侧倾、贴卡歪斜带来的形变一并抵消掉。

    ★ v17: img 现在传【灰度图】。warpPerspective 本来就不关心通道数, 搬 1 通道
      比搬 3 通道少 2/3 的内存带宽(1.28ms → 1.04ms), 而下游 detect_cross_center
      和 shape_from_roi 原本拿到彩色图后第一件事就是各自 cvtColor 一次 ——
      那两次转换现在直接消失了。彩色矫正图只有要画右上角缩略图时才单独算一次,
      比赛模式(SHOW_LEVEL=0)下一次都不算。实测分类结果与原版逐张一致。
    """
    d = CARD_PX - 1
    dst = np.array([[0, 0], [d, 0], [d, d], [0, d]], np.float32)
    M = cv2.getPerspectiveTransform(quad, dst)
    return cv2.warpPerspective(img, M, (CARD_PX, CARD_PX))


# ============================ 3. 十字中心特征 + 形状提取 ============================
# 这一节是本版优化的重头戏。判据一个字没改: 还是"中心有黑点 + 上下左右四条臂
# 都连续"。改的只是把 Python 双重循环换成整幅矩阵运算。
#
# 原版的算法结构(保留在 _cross_center_ref 里, 一字未动):
#   for cy in 7 个候选行:
#     for cx in 7 个候选列:
#        中心小块均值 < 0.12 → 跳过
#        左右臂: 在 cy±band 的横带里, 对每一列 any(), 再求均值
#        上下臂: 在 cx±band 的竖带里, 对每一行 any(), 再求均值
#        best = max(best, min(四条臂))
# 一共 49 个候选 x 4 条臂 = 196 次 np.any 窗口归约, 每次都要新开一块临时数组。
# 实测 8.48ms —— 整个流水线最贵的单项, 比 4 张全分辨率掩码加起来还贵。
#
# 向量化的三步(结果与逐点扫描**逐值相等**):
#   1. "cy±band 这一列里有没有墨" = 对 ink 做 (2*band+1, 1) 的**最大值滤波**,
#      也就是 cv2.dilate。膨胀后 H[y,x] 恰好等于原版 any(ink[y-band:y+band+1, x])。
#      竖直方向同理用 (1, 2*band+1) 的核得到 V。这一步把 196 次 any 变成 2 次 dilate。
#   2. 臂上的"连续比例"是定长窗口求和 → 用 cumsum 前缀和 O(1) 取任意区间。
#   3. 中心小块均值 → cv2.integral 积分图, 同样 O(1)。
#   4. 49 个候选中心一次性用 numpy 广播算完, 不再有 Python 循环。
# 实测 6.78ms → 0.95ms (7.1x)。
#
# ★ 唯一的行为差异, 说清楚: 原版有 `if best >= 0.96: break` 提前退出, 所以它
#   返回的是**扫描顺序里第一个**超过 0.96 的候选; 向量化版返回的是**真正的
#   最大值**。两者只在"已经确认是十字"之后的小数点后几位上不同(实测 1.0000
#   vs 0.9694 这种)。做过 300 组随机图卡对拍: 判定结果(conf>0 → 十字)
#   **0 次翻转**, 且向量化版的 conf **从未低于**原版 —— 投票门限 VOTE_CONF=0.45
#   离这个区间很远, 不可能因此改判。
def _cross_center_ref(ink):
    """原版逐点扫描实现, **一字未改**, 只用于真机 A/B 对拍和源码核验。

    CROSS_FAST=False 时走这一支。正常比赛不要用 —— 它比向量化版慢 7 倍。
    """
    h, w = ink.shape
    if h < 40 or w < 40:
        return 0.0
    density = float(np.mean(ink))
    if density < 0.01 or density > 0.42:
        return 0.0                         # 空白或大块实心图形
    side = min(h, w)
    start = max(4, int(side * 0.10))
    end = int(side * 0.38)
    band = max(2, int(side * 0.055))      # 允许少量倾斜和透视残差
    center_r = max(2, int(side * 0.035))
    step = max(2, side // 55)
    best = 0.0
    for cy in range(int(h * 0.35), int(h * 0.65) + 1, step):
        if cy - end < 0 or cy + end >= h:
            continue
        for cx in range(int(w * 0.35), int(w * 0.65) + 1, step):
            if cx - end < 0 or cx + end >= w:
                continue
            center = ink[cy-center_r:cy+center_r+1,
                         cx-center_r:cx+center_r+1]
            if float(np.mean(center)) < 0.12:
                continue                    # 其他空心图形中心没有黑色交点
            horizontal = ink[cy-band:cy+band+1, :]
            vertical = ink[:, cx-band:cx+band+1]
            arms = (
                float(np.mean(np.any(horizontal[:, cx-end:cx-start], axis=0))),
                float(np.mean(np.any(horizontal[:, cx+start:cx+end], axis=0))),
                float(np.mean(np.any(vertical[cy-end:cy-start, :], axis=1))),
                float(np.mean(np.any(vertical[cy+start:cy+end, :], axis=1))),
            )
            best = max(best, min(arms))
            if best >= 0.96:
                break
        if best >= 0.96:
            break
    if best < 0.70:
        return 0.0
    return min(1.0, 0.55 + 0.45 * (best - 0.70) / 0.30)


def cross_center_from_binary(ink):
    """在矫正图卡的墨迹图中找黑色交点和连续的上下左右四臂。

    返回 0 表示不是十字；正数是 0~1 置信度。独立于轮廓面积/顶点数，
    因此细线十字被形态学开运算擦掉时仍能识别。

    参数 ink: bool 或 0/1 的墨迹图(True = 黑墨)。
    """
    if not CROSS_FAST:
        return _cross_center_ref(ink)

    h, w = ink.shape
    if h < 40 or w < 40:
        return 0.0
    density = float(np.mean(ink))
    if density < 0.01 or density > 0.42:
        return 0.0                         # 空白或大块实心图形

    # ---- 以下 6 个尺度参数与原版逐字相同, 不要动 ----
    side = min(h, w)
    start = max(4, int(side * 0.10))
    end = int(side * 0.38)
    band = max(2, int(side * 0.055))      # 允许少量倾斜和透视残差
    center_r = max(2, int(side * 0.035))
    step = max(2, side // 55)
    arm_len = end - start                 # 每条臂上参与统计的像素列数
    if arm_len <= 0:
        return 0.0

    u = ink.astype(np.uint8)              # dilate/integral 要 uint8

    # 第 1 步: 最大值滤波 == 原版的 np.any 窗口归约
    #   Hc[y, x] = 1 ⟺ ink[y-band : y+band+1, x] 里有墨
    #            == 原版 np.any(horizontal[:, x], axis=0), 当 cy == y 时
    #   Vr[y, x] = 1 ⟺ ink[y, x-band : x+band+1] 里有墨
    Hc = cv2.dilate(u, np.ones((2 * band + 1, 1), np.uint8))
    Vr = cv2.dilate(u, np.ones((1, 2 * band + 1), np.uint8))

    # 第 2 步: 前缀和, 让"任意区间内有墨的列数"变成两次查表相减
    cs_h = np.zeros((h, w + 1), np.int32)
    np.cumsum(Hc, axis=1, out=cs_h[:, 1:])          # 沿 x 累加, 给左右臂用
    cs_v = np.zeros((h + 1, w), np.int32)
    np.cumsum(Vr, axis=0, out=cs_v[1:, :])          # 沿 y 累加, 给上下臂用

    # 第 3 步: 积分图, 让中心小块均值变成四次查表
    integ = cv2.integral(u)                          # (h+1, w+1), int32

    # 第 4 步: 所有候选中心一次算完(原版是 7x7 的双重 Python 循环)
    ys = np.arange(int(h * 0.35), int(h * 0.65) + 1, step)
    xs = np.arange(int(w * 0.35), int(w * 0.65) + 1, step)
    ys = ys[(ys - end >= 0) & (ys + end < h)]        # == 原版的 continue 边界检查
    xs = xs[(xs - end >= 0) & (xs + end < w)]
    if ys.size == 0 or xs.size == 0:
        return 0.0
    Y = ys[:, None]
    X = xs[None, :]

    r = center_r
    cen = (integ[Y + r + 1, X + r + 1] - integ[Y - r, X + r + 1]
           - integ[Y + r + 1, X - r] + integ[Y - r, X - r]) / float((2 * r + 1) ** 2)

    arm_l = cs_h[Y, X - start] - cs_h[Y, X - end]    # 左臂: [cx-end, cx-start)
    arm_r = cs_h[Y, X + end] - cs_h[Y, X + start]    # 右臂: [cx+start, cx+end)
    arm_u = cs_v[Y - start, X] - cs_v[Y - end, X]    # 上臂
    arm_d = cs_v[Y + end, X] - cs_v[Y + start, X]    # 下臂
    arms = np.minimum(np.minimum(arm_l, arm_r),
                      np.minimum(arm_u, arm_d)) / float(arm_len)
    arms = np.where(cen >= 0.12, arms, 0.0)          # 中心没黑点的候选作废
    best = float(arms.max())

    if best < 0.70:
        return 0.0
    return min(1.0, 0.55 + 0.45 * (best - 0.70) / 0.30)


def detect_cross_center(card_gray):
    """先裁掉图卡黑边框, 再查中心四臂。入参是**灰度**矫正图(v17 起)。"""
    h, w = card_gray.shape[:2]
    margin = max(2, int(min(h, w) * 0.10))
    roi = card_gray[margin:h-margin, margin:w-margin]
    _, ink = cv2.threshold(roi, 0, 255,
                           cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    return cross_center_from_binary(ink != 0)


def shape_from_roi(roi):
    """在卡片 ROI 里提出形状外轮廓。返回 (cnt, 二值图)。入参是**灰度** ROI。"""
    g = cv2.GaussianBlur(roi, (5, 5), 0)
    _, b = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)  # 黑墨=255
    if b.mean() < 6:
        return None, b                              # ROI 里几乎没墨迹 → 卡片没对准
    b = cv2.morphologyEx(b, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    # ★ 剔除贴着 ROI 边缘的连通域 —— 那是卡片自己的黑边框被 warp 带进来的残渣。
    #   不干掉它, 它会以"正方形"的身份被报上去。
    # (这里的 `keep[lab == i] = 255` 看着像能用查表 lut[lab] 一次替掉, 实测反而
    #  更慢: 图卡 ROI 上连通域通常只有 1~2 个, 逐个比较比对整幅 int32 标签图
    #  做一次 gather 便宜。0.043ms vs 0.279ms, 所以**维持原样**。)
    H, W = b.shape
    n, lab, stats, _ = cv2.connectedComponentsWithStats(b, 8)
    keep = np.zeros_like(b)
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1:
            continue
        keep[lab == i] = 255
    if keep.max() == 0:
        return None, b                              # 只剩边框, 没有形状

    cnts, _ = cv2.findContours(keep, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, b
    cnt = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(cnt) < 0.015 * H * W:
        return None, b                              # 太小, 估计是噪点
    return cnt, keep


def extract_shape(card_gray):
    """内缩切掉卡片边框后提形状; 第一次没提到就加大内缩重试。入参是灰度矫正图。"""
    last = None
    for ins_r in INSET_RETRY:
        ins = int(CARD_PX * ins_r)
        roi = card_gray[ins:CARD_PX - ins, ins:CARD_PX - ins]
        cnt, b = shape_from_roi(roi)
        if cnt is not None:
            return cnt, b, ins_r
        last = b
    return None, last, INSET_RETRY[-1]


# ============================ 4. 形状特征 ============================
def order_pts(ap):
    """把多边形顶点按绕质心角度排成一致顺序(用于数凹顶点)。"""
    p = np.asarray(ap, np.float32).reshape(-1, 2)
    if len(p) < 3:
        return p
    c = p.mean(axis=0)
    return p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]


def reflex_count(ap):
    """数【凹顶点】个数 —— 区分五角星(5个)和十字形(8个)的主力特征。

    判据: 沿多边形走一圈, 某顶点的转向与整体绕向相反, 该顶点就是凹的。
    比 convexityDefects 稳: 后者依赖凸包的边结构, 十字形的凸包只有 4 条边,
    每条边要承担两个凹口, 数量容易数不准; 而"数转向反转处"直接看轮廓本身。
    """
    p = order_pts(ap)
    n = len(p)
    if n < 4:
        return 0
    a = 0.0
    for i in range(n):
        x1, y1 = p[i]
        x2, y2 = p[(i + 1) % n]
        a += x1 * y2 - x2 * y1                      # 有向面积 → 整体绕向
    sgn = 1.0 if a > 0 else -1.0
    cnt = 0
    for i in range(n):
        v1 = p[(i + 1) % n] - p[i]
        v2 = p[(i + 2) % n] - p[(i + 1) % n]
        if (v1[0] * v2[1] - v1[1] * v2[0]) * sgn < -1e-6:
            cnt += 1                                # 转向反转 → 凹顶点
    return cnt


def min_edge_angle(ap):
    """四边形各边相对"卡片水平/垂直轴"的最小夹角, 0~45°。

    → 正方形: 边平行于卡片边框 → ≈0°
    → 菱形  : 边与卡片边框成 45° → ≈45°
    这是区分正方形/菱形的**唯一**可靠依据 —— Hu 矩旋转不变, 先天分不出这两个。
    """
    if len(ap) != 4:
        return None
    p = ap.reshape(4, 2).astype(np.float32)
    best = 45.0
    for i in range(4):
        dx, dy = p[(i + 1) % 4] - p[i]
        a = math.degrees(math.atan2(dy, dx)) % 90.0
        best = min(best, min(a, 90.0 - a))
    return best


def features(cnt):
    """几何轮廓特征。顶点用【凸包拟合】, 抗断裂也抗圆角。"""
    area = cv2.contourArea(cnt)
    peri = cv2.arcLength(cnt, True)
    if area < 1 or peri < 1e-3:
        return None

    # ★ 凸包只算一次。原版这里算点集 convexHull(cnt), 下面数深凹口时又算了一次
    #   索引版 convexHull(cnt, returnPoints=False) —— 同一个凸包算两遍。
    #   现在先拿索引, 点集直接按索引取: hull = cnt[hull_idx[:, 0]], 两者逐点相同
    #   (np.array_equal 验证通过, 面积差 0.000000)。0.033ms → 0.021ms。
    hull_idx = cv2.convexHull(cnt, returnPoints=False)
    hull = cnt[hull_idx[:, 0]] if hull_idx is not None and len(hull_idx) else cv2.convexHull(cnt)
    ha = max(cv2.contourArea(hull), 1e-6)
    hp = cv2.arcLength(hull, True)

    f = {
        "area": area,
        "circ": 4 * math.pi * area / (peri * peri),      # 圆度: 圆1.0 方0.785 三角0.605
        "solid": area / ha,                              # 凸性: 凹形 <0.88
    }
    (_, _), (rw, rh), rang = cv2.minAreaRect(cnt)
    f["fill"] = area / max(rw * rh, 1e-6)                # 填充率: 方1.0 圆0.785 星0.45 十字0.25
    a_rect = abs(((rang % 90) + 90) % 90)
    f["ang_rect"] = min(a_rect, 90 - a_rect)

    hull_ap = cv2.approxPolyDP(hull, 0.03 * hp, True)
    f["v"] = len(hull_ap)                                # 凸包顶点: 三角3 方/菱/十字4 星5
    f["edge"] = min_edge_angle(hull_ap)

    raw_ap = cv2.approxPolyDP(cnt, 0.02 * peri, True)
    f["reflex"] = reflex_count(raw_ap)                   # 凹顶点: 星5 十字8 其余0

    depths = []
    try:
        hi = hull_idx
        if hi is not None and len(hi) > 3:
            dfx = cv2.convexityDefects(cnt, hi)
            if dfx is not None:
                depths = [dfx[i, 0, 3] / 256.0 for i in range(dfx.shape[0])]
    except cv2.error:
        pass
    f["deep"] = sum(1 for x in depths if x > DEFECT_MIN * CARD_PX)
    return f


# ============================ 5. 判定 ============================
def _near(x, ideal, tol):
    """连续量的贴合度: 完全吻合=1, 差到 tol 以外=0。"""
    return max(0.0, 1.0 - abs(x - ideal) / tol)


def _near_int(x, ideal, tol):
    """离散量(顶点数/凹顶点数)的贴合度: 容差内满分, 之后每个差 0.45。"""
    d = abs(x - ideal) - tol
    return 1.0 if d <= 0 else max(0.0, 1.0 - 0.45 * d)


def score_all(f):
    """给 6 个形状各打一个 0~1 的分。返回 {形状: 得分}。

    用打分而不是硬 if 树: 某一项特征测不准时只扣分、不直接把形状判死,
    现场光照/距离变化下比阈值树稳得多, 也方便直接打印出来调参。
    """
    scores = {}
    for name, p in PROFILES.items():
        num = den = 0.0
        num += W["circ"] * _near(f["circ"], p["circ"], TOL["circ"]);        den += W["circ"]
        num += W["solid"] * _near(f["solid"], p["solid"], TOL["solid"]);    den += W["solid"]
        num += W["fill"] * _near(f["fill"], p["fill"], TOL["fill"]);        den += W["fill"]
        num += W["v"] * _near_int(f["v"], p["v"], p["v_tol"]);              den += W["v"]
        num += W["reflex"] * _near_int(f["reflex"], p["reflex"], p["reflex_tol"]); den += W["reflex"]
        num += W["deep"] * _near_int(f["deep"], p["deep"], p["deep_tol"]);  den += W["deep"]
        scores[name] = num / max(den, 1e-9)
    return scores


def hu_pick(cnt, tpl):
    """Hu 矩兜底判定。返回 (形状名, 置信度0~1, 全部距离)。

    ★ 关键: 计算"最佳/次佳"比值时必须排除【旋转等价对】(正方形/菱形)。
      菱形就是正方形旋转 45°, Hu 矩旋转不变 → 两者距离几乎相同,
      拿它们互相当次佳, 比值恒 ≈1.0, 会把真正的正确结果压成"不可信"。
    """
    d = {k: float(cv2.matchShapes(cnt, c, cv2.CONTOURS_MATCH_I2, 0.0)) for k, c in tpl.items()}
    order = sorted(d, key=d.get)
    best = order[0]
    pool = [k for k in order
            if k != best and not (k in ROT_TWIN and best in ROT_TWIN)]
    best_other = min((d[k] for k in pool), default=1e9)
    ratio = best_other / max(d[best], 1e-9)
    conf = max(0.0, min(1.0, (ratio - 1.0) / 2.0))   # 比值 1→0, 3→1
    return best, conf, d


def classify(cnt, f, tpl):
    """几何打分主判定 + Hu 矩兜底。返回 (形状名, 置信度, 得分表)。"""
    scores = score_all(f)

    # 四顶点凸轮廓已经排除了圆形。菱形在当前相机距离下圆度约 0.72，
    # 会与圆形模板打成近似平局；此时直接使用矫正图中的边方向区分正方形/菱形。
    # 十字在进入 classify() 前已由中心四臂判据处理，不受这里影响。
    ang = f["edge"] if f["edge"] is not None else f["ang_rect"]
    if (f["v"] == 4 and f["reflex"] == 0 and f["deep"] == 0
            and ang is not None):
        name = "square" if ang < ANG_TOL else "diamond"
        conf = 0.5 + 0.5 * min(1.0, abs(ang - ANG_TOL) / ANG_TOL)
        return name, max(scores[name], conf), scores

    order = sorted(scores, key=scores.get, reverse=True)
    a, b = order[0], order[1]

    # ---- 正方形 vs 菱形: 几何打分天然打平, 必须靠边方向分 ----
    if {a, b} == set(ROT_TWIN):
        ang = f["edge"] if f["edge"] is not None else f["ang_rect"]
        a = "square" if ang < ANG_TOL else "diamond"
        b = "diamond" if a == "square" else "square"
        conf = 0.5 + 0.5 * min(1.0, abs(ang - ANG_TOL) / ANG_TOL)
        return a, conf, scores

    conf = scores[a]

    # ---- 几何打分整体偏低 → 请 Hu 矩兜底(仅作参考, 不打压几何结论) ----
    if conf < SCORE_MIN:
        hname, hconf, hd = hu_pick(cnt, tpl)
        # 只有在 Hu 矩距离明显更优、且不与几何结论冲突时才采纳
        if hconf > conf and hname not in ROT_TWIN:
            return hname, hconf, scores
    return a, conf, scores


# ============================ 5.5 快筛 + 循线 ============================
def card_likely(gray):
    """1/4 小图快筛: 这一帧里到底有没有"方块状暗团"。没有就整条图形流水线都不跑。

    图形流水线最贵的部分是 ink_masks() 的全分辨率掩码 + 闭运算 + findContours。
    这里用 1/16 的像素量先过一遍, 空帧直接省掉 —— 这是把帧率从 2fps 拉回来的关键。
    判据故意放得比正式流水线松: 快筛放过一个只是多花一帧的时间, 错杀一个就漏卡了。

    ★ v17: 入参从 bgr 改成 main 已经算好的 gray。原来是先把 640x480x3 缩成
      160x120x3 再 cvtColor, 等于对三通道做了一遍面积插值; 现在直接缩单通道,
      省掉 2/3 的插值工作量和一次 cvtColor。1.17ms → 0.66ms。
      ★ 这两条路径的灰度值不是逐位相同的(先缩后转 vs 先转后缩, 差 ±1 灰阶,
        因为 BGR→GRAY 的定点舍入发生在不同环节)。所以单独验过决策一致性:
        120 帧实拍/合成混合样本, True/False **120/120 完全一致**(其中 68 帧为
        True)。能一致是因为这里的判据全是面积/长宽比/填充率这类整体量, ±1 灰阶
        动不了 Otsu 的分界, 更动不了连通域的形状。
    """
    small = cv2.resize(gray, (GATE_W, GATE_H), interpolation=cv2.INTER_AREA)
    _, b = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    b = cv2.morphologyEx(b, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(b, 8)
    total = GATE_W * GATE_H
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if a < 0.003 * total or a > AREA_MAX_REL * total or w < 4 or h < 4:
            continue
        if not (0.40 < w / float(h) < 2.50):        # 斜视角下卡片是梯形, 放很宽
            continue
        # 空心方框的连通域只是个"环", 填充率天然只有 0.2 左右, 下限必须压到 0.10
        if a / float(w * h) < 0.10:
            continue
        return True
    return False


def _runs(mask):
    """布尔序列里每段连续 True 的 (中心, 宽度, 起点, 终点)。

    两端各垫一个 0 再做差分, 一次 flatnonzero 拿到所有段的起止,
    不用在 640 个点上写 Python 循环 —— 纠偏每帧都要跑, 这里省的是真金白银。
    """
    m = np.concatenate(([0], mask.astype(np.int8), [0]))
    d = np.diff(m)
    s = np.flatnonzero(d == 1)
    e = np.flatnonzero(d == -1)
    return [((a + b - 1) * 0.5, int(b - a), int(a), int(b - 1))
            for a, b in zip(s, e)]


def dark_segments(prof, thr):
    """一维亮度剖面上的所有暗段, [(中心x, 宽度, 起, 止), ...]。"""
    return _runs(prof < thr)


def quad_x_span(quad, y):
    """四边形与水平线 y 的交点 x 区间 (x0, x1); 不相交返回 None。

    用于把图卡占的那段从扫描带里挖掉 —— 图卡的黑边框和赛道黑线在灰度剖面里
    长得一模一样, 但图形流水线已经把卡片轮廓算出来了, 直接拿来用最省也最准。
    """
    if quad is None:
        return None
    p = np.asarray(quad, np.float32).reshape(-1, 2)
    xs = []
    n = len(p)
    for i in range(n):
        x1, y1 = p[i]
        x2, y2 = p[(i + 1) % n]
        if y1 == y2 or (y1 - y) * (y2 - y) > 0:     # 这条边不跨越 y
            continue
        xs.append(x1 + (y - y1) / (y2 - y1) * (x2 - x1))
    if len(xs) < 2:
        return None
    return min(xs), max(xs)


def card_zones(prof, med):
    """找出这条带上属于图卡的 x 区间(白底纸 + 落在它里面的黑边框/黑图形)。

    判据: 图卡是白底纸, 比赛道面亮一截, 而且整块有相当宽度。赛道黑线旁边没有
    这么一大片白。所以凡是落在白块里的暗段都归给图卡。

    扫描带穿过图形时, 图形的墨会把白纸切成几条窄白条, 所以先用一维闭运算
    (跨度 CARD_INK_MAX)把它们接回成一整块再量宽度 —— 直接量单条白段会漏。
    """
    bright = (prof > med + CARD_WHITE_DN).astype(np.uint8)
    if bright.sum() * 2 > bright.size:      # 整带过半偏白: 白不再是图卡的标志
        return []
    k = int(CARD_INK_MAX) * 2 + 1
    closed = cv2.morphologyEx(bright.reshape(1, -1), cv2.MORPH_CLOSE,
                              np.ones((1, k), np.uint8)).ravel()
    return [(s - CARD_PAD, e + CARD_PAD)
            for _, wd, s, e in _runs(closed > 0)
            if CARD_WHITE_MIN <= wd <= CARD_W_MAX]


# ============================ 5.6 离散纠偏状态机 ============================
# 对应流程图: 识别赛道 → 检测左右黑线 → 算前方中心 → e=x_far-中线
#            → |e|>门限? 否→前进1~2步 / 是→停→定方向→原地转3°→复查
#
# 用横向扫描带而不是全图二值化+findContours: 两条带一共才几千个像素,
# 一次 mean 压成一维剖面就能定位赛道边, 比完整轮廓流水线便宜一个数量级。
# 省下来的时间才够图形识别用。

CMD_HOLD, CMD_FWD, CMD_TURN_L, CMD_TURN_R, CMD_BLOCKED, CMD_HOME = range(6)
# 5保留原HOME，不发送它；STM32的5为可选BACK。新增平移使用6、7。
CMD_SHIFT_L, CMD_SHIFT_R = 6, 7
CMD_NAME = {CMD_HOLD: "HOLD", CMD_FWD: "FWD", CMD_TURN_L: "TURNL",
            CMD_TURN_R: "TURNR", CMD_BLOCKED: "BLOCKED", CMD_HOME: "HOME",
            CMD_SHIFT_L: "SHIFTL", CMD_SHIFT_R: "SHIFTR"}


def pick_edges(segs, cx, c_prior, half):
    """把暗段分成左右两条车道线, 返回 (xl, xr); 没认出来的那侧为 None。

    c_prior: 上一帧量到的赛道中心(None = 本带还没成功量到过)。
    half:    赛道半宽先验。

    关键在于**不用画面中线分左右** —— 见 EDGE_NEED_PRIOR 那段注释。
    """
    if c_prior is None:
        # 冷启动: 没有历史可依, 只能拿画面中线分。对"左右都在画面里"这种情况
        # 它永远是对的, 所以足够用来把历史建立起来; 真正不能信的是只看到一条。
        left = [s for s in segs if s[0] < cx]
        right = [s for s in segs if s[0] >= cx]
        # 取"最靠内侧"的那条 —— 画面里可能同时有本圈的线和回弯段的线,
        # 紧贴机器人两侧的才是当前要跟的赛道边。
        xl = max(left, key=lambda s: s[0])[0] if left else None
        xr = min(right, key=lambda s: s[0])[0] if right else None
        if EDGE_NEED_PRIOR and (xl is None) != (xr is None):
            return None, None          # 只看到一条又没有历史 → 宁可报丢线也不猜
        return xl, xr

    # 有历史: 两条车道线该出现在 c_prior ± half。每条边各取离它的预测位置
    # 最近的那个暗段, 离得太远的一概不认(刻度数字、虚线、阴影都在这里被挡掉)。
    pl, pr = c_prior - half, c_prior + half
    tol = half * EDGE_GATE_R
    cl = [(abs(s[0] - pl), s[0]) for s in segs if abs(s[0] - pl) <= tol]
    cr = [(abs(s[0] - pr), s[0]) for s in segs if abs(s[0] - pr) <= tol]
    xl = min(cl)[1] if cl else None
    xr = min(cr)[1] if cr else None

    # 同一个暗段同时被两边选中(机器人几乎压在某条线上) → 判给更贴合的那边,
    # 另一边当没看到, 免得算出零或负的半宽。
    if xl is not None and xr is not None and xl >= xr:
        if abs(xl - pl) <= abs(xr - pr):
            xr = None
        else:
            xl = None
    return xl, xr


def track_center(gray, yc, w, half_prior, quad=None, center_prior=None):
    """在 yc 这条带上找左右两条黑线, 返回 (中心x, 半宽, 标志, 图卡区间)。

    标志: "LR" 左右都找到 / "L" 只有左边 / "R" 只有右边 / "" 全丢。
    图卡区间: 本帧被判为图卡而剔除的 [(x0,x1), ...], 只用于画面叠加排查。

    只看到一条边时用 half_prior 外推 —— 这不是凑数: 过弯时内侧那条线会
    整条滑出画面, 此时如果直接判"丢线停车", 机器人会卡在每个弯道上。
    赛道宽度是固定的, 用它外推比停下来可靠得多。

    quad: 本帧已定位的图卡四边形(可为 None)。它覆盖的那段 x 会被挖掉,
          否则图卡的黑边框会被当成赛道黑线, 把 e 带偏。
    center_prior: 上一帧【这条带】量到的赛道中心(None = 还没量到过)。
          用它来判"只看到的这一条线是左是右" —— 不能拿画面中线判, 过弯时
          标签会翻。两条带的中心不一样(透视 + 弯道), 所以必须各记各的。
    """
    h = gray.shape[0]
    y0 = max(0, int(yc - BAND_H * 0.5))
    y1 = min(h, y0 + BAND_H)
    if y1 - y0 < 2:
        return None, half_prior, "", []

    # 剖面 = 这条带每一列的平均灰度。cv2.reduce 走 SIMD 且直接出 float32,
    # 比 ndarray.mean(axis=0) 先升 float64 再归约快 3 倍(0.036ms → 0.010ms),
    # 两者最大差 0.000 灰阶。每帧两条带, 省得不多, 但是白捡的。
    prof = cv2.reduce(gray[y0:y1, :], 0, cv2.REDUCE_AVG,
                      dtype=cv2.CV_32F).ravel()

    # ---- 第一层: 用已定位的图卡四边形直接挖掉那段 ----
    hard = []                                        # 第一层: 几何已验证, 可信
    # 独立防线：外部/旧缓存传入的无效大框也不能遮掉整个巡线带。
    span = (quad_x_span(quad, (y0 + y1) * 0.5)
            if quad is not None and not card_quad_reject_reason(quad, w, h) else None)
    if span is not None:
        hard.append((span[0] - CARD_PAD, span[1] + CARD_PAD))

    # 阈值用的中位数要在挖掉图卡之后再算: 一大片白底纸会把中位数抬高,
    # 连带把阈值抬高, 赛道黑线反而更容易被切出来 —— 那是侥幸, 不是设计。
    keep = np.ones(prof.shape[0], bool)
    for a, b in hard:
        keep[max(0, int(a)):max(0, int(b) + 1)] = False
    if keep.sum() < prof.shape[0] * 0.25:            # 图卡几乎占满这条带
        return None, half_prior, "", []
    med = float(np.median(prof[keep]))

    # ---- 第二层: 四边形没拟合上时, 靠"白底纸"把图卡认出来 ----
    soft = card_zones(prof, med) if CARD_REJECT_ON else []

    thr = med - LINE_OFF
    if thr < 8:
        return None, half_prior, "", []

    kept, loose = [], []
    for c, wd, s, e in dark_segments(prof, thr):
        if not (LINE_MIN_W <= wd <= LINE_MAX_W):
            continue
        if any(s <= b and e >= a for a, b in hard):  # 与第一层图卡区间重叠
            continue
        loose.append((c, wd))
        if any(s <= b and e >= a for a, b in soft):  # 与第二层白底纸区间重叠
            continue
        kept.append((c, wd))

    # 第二层把线全吃掉了 → 它判错的可能性大于真的一条线都没有, 退回只信第一层。
    # (bad 只用来画叠加层, 所以照样把 soft 带出去 —— 屏幕上能看见第二层被否决了)
    segs = kept if kept else loose
    bad = hard + soft
    if not segs:
        return None, half_prior, "", bad

    cx = w * 0.5
    xl, xr = pick_edges(segs, cx, center_prior, half_prior)

    if xl is not None and xr is not None:
        half = (xr - xl) * 0.5
        if half < LINE_MIN_W:                       # 两段贴在一起 = 其实是同一条线
            return (xl + xr) * 0.5, half_prior, "LR", bad
        return (xl + xr) * 0.5, half, "LR", bad
    if xl is not None:
        return xl + half_prior, half_prior, "L", bad
    if xr is not None:
        return xr - half_prior, half_prior, "R", bad
    return None, half_prior, "", bad


class LineCorrector:
    """走-停-转-复查 离散纠偏。随时可调用, 自带节拍控制。

    用法一(嵌在主循环里, 非阻塞):
        cmd, info = corr.step(gray)
        if cmd is not None:  out.send_cmd(cmd, info["arg"])

    用法二(需要先对准再做别的事, 阻塞直到对准):
        ok = corr.align(cam, out, timeout_s=8.0)
    """

    # 状态: "MEASURE" 可以测量并决策 / "WAIT" 上一条指令还在执行
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.cx = w * 0.5
        self.half = CORR_TRACK_W * 0.5 * (w / 640.0)
        self.e_tol = CORR_E_TOL * (w / 640.0)       # 门限随分辨率缩放,
        self.e_ok  = CORR_E_OK * (w / 640.0)        # 换 320x240 不用重调参
        self.wait = 0
        self.turns = 0          # 本轮已连续转了多少次
        self.net_turn = 0       # 本轮累计净转角(度, 右正左负) —— 丢线时按它回正
        self.good = 0           # 连续对准的拍数, 到 CORR_GOOD_N 才清 net_turn
        self.lost = 0           # 连续丢线帧数
        self.homed = False      # 本次丢线已经发过回正序列, 不重复发
        self.c_far = None       # 上一帧【远带】量到的赛道中心 —— 判左右用
        self.c_near = None      # 近带同上。两条带的中心不一样, 必须分开记
        self.pending = []       # 待发指令队列 [(cmd, arg, wait)], 一帧发一条
        self.aligned = False
        self.last_e = 0.0
        self.near_e = None
        self.heading_e = None
        self.correction_active = False
        self.flag = ""
        self.pts = []
        self.zones = []         # 本帧判为"图卡"的 x 区间, 只用于画面叠加

    def measure(self, gray, quad=None):
        """只测量不决策。返回 (e, 标志); 丢线时 e 为 None。

        e = x_far - 画面中线。流程图写的 "x_far-160" 是 320 宽下的中线,
        这里用 w/2, 640 宽下就是 320 —— 换分辨率不用改代码。

        quad: 本帧图卡四边形, 传进去让扫描带把图卡边框挖掉。
        """
        x_far, half, fl, zones = track_center(gray, self.h * CORR_BAND, self.w,
                                              self.half, quad, self.c_far)
        # 近带只用来更新赛道宽先验: 近处透视畸变小、两条边都在画面内的概率高,
        # 它量出来的宽度比远带可靠, 拿它去喂远带的外推。
        x_near, half_n, fl_n, _ = track_center(gray, self.h * CORR_BAND_N, self.w,
                                               self.half, quad, self.c_near)
        # 只用本帧同时可见的两条边决定平移；不拿上一帧中心充当新测量。
        self.near_e = None if x_near is None else x_near - self.cx
        self.heading_e = (x_far - x_near) if (
            x_far is not None and x_near is not None and fl == "LR" and fl_n == "LR") else None
        if fl_n == "LR":
            self.half = 0.7 * self.half + 0.3 * half_n   # 低通, 防止单帧误测带偏
        elif fl == "LR":
            self.half = 0.7 * self.half + 0.3 * half

        # 量到了就更新各自的中心先验, 下一帧靠它分左右。
        # 单帧丢线【不清】先验: 一脚踩过线、反光、过弯内侧线出画都会造成单帧丢,
        # 这时候把历史扔掉, 下一帧又退回"拿画面中线猜"的老路了。
        # 真正的清除放在 step() 里确认丢线那一支, 以及 align() 开头。
        if x_far is not None:
            self.c_far = x_far
        if x_near is not None:
            self.c_near = x_near

        self.flag = fl
        self.zones = zones
        self.pts = []
        if x_far is None:
            return None, fl
        self.pts.append((x_far, self.h * CORR_BAND))
        e = x_far - self.cx
        self.last_e = e
        return e, fl

    def _emit(self):
        """从待发队列取一条发出去, 顺带按它自己的节拍设置等待。"""
        cmd, arg, wait = self.pending.pop(0)
        self.wait = wait
        return cmd, {"e": self.last_e, "arg": arg, "flag": self.flag,
                     "turns": self.turns}

    def resume(self):
        """立刻恢复测量 —— 用在"上报形状后马上转入循线模式"这种场合。

        step() 开头那句 `if self.wait > 0` 会把剩下的节拍先耗完才重新测量。
        识别图卡那几帧里纠偏多半刚发过一条 FWD/TURN, 残留着 CORR_SETTLE 帧的
        等待; 不清掉的话"马上转循线"要再等六帧才真的动, 就不算马上了。

        只清 wait, **不动** pending: pending 里装的是丢线回正序列, 那串指令
        半路打断会把机器人停在一个两不靠的朝向上(step() 里有同样的注释)。
        turns / net_turn 也不动 —— 累计转角还要留着给丢线回正用。
        """
        self.wait = 0

    def step(self, gray, quad=None):
        """每帧产生新巡线建议；动作执行和周期末投票由STM32负责。

        不把发送次数当作已执行转向次数，不排累计角度回转或HOME指令。
        HOLD/BLOCKED为停止建议，STM32在当前周期末落脚收脚。
        """
        e, fl = self.measure(gray, quad)
        info = {"e": 0.0 if e is None else e, "arg": 0,
                "flag": fl, "turns": 0}
        if e is None:
            self.aligned = False
            self.good = 0
            self.lost += 1
            if self.lost == CORR_LOST_N:
                # 恢复原有确认丢线后清先验：新姿态不能沿旧弯道中心误配左右边。
                self.c_far = None
                self.c_near = None
            return (CMD_HOLD if self.lost < CORR_LOST_N else CMD_BLOCKED), info

        self.lost = 0
        self.homed = False
        thr = self.e_ok if self.correction_active else self.e_tol
        if abs(e) <= thr:
            self.good += 1
            self.aligned = True
            self.correction_active = False
            self.turns = 0
            info["arg"] = CORR_STEP_N
            return CMD_FWD, info

        self.aligned = False
        self.good = 0
        self.correction_active = True
        self.turns = 1  # 仅表示正在纠偏，不能累计成实机转向次数。
        scale = self.w / 640.0
        if (CORR_SHIFT_ON and self.heading_e is not None and self.near_e is not None
                and abs(self.heading_e) <= CORR_HEADING_TOL * scale
                and abs(self.near_e) > thr and self.near_e * e > 0):
            info["arg"] = 1  # 一次平移动作；距离由已调动作表决定。
            return (CMD_SHIFT_R if self.near_e > 0 else CMD_SHIFT_L), info

        info["arg"] = CORR_TURN_DEG  # 仅保留协议字段，STM32只执行一次约30度转向。
        return (CMD_TURN_R if e > 0 else CMD_TURN_L), info

    def align(self, cam, out, timeout_s=8.0, disp=None):
        """阻塞版: 反复纠偏直到对准(或超时)。随时可调用。

        返回 True = 已对准。想在识别到图卡、做完肢体动作之后重新找正,
        直接调这一个函数就行, 不用自己写循环。
        注意它只发 TURN/HOME/HOLD, 不发 FWD —— 纯"原地找正", 不会把机器人走偏。
        丢线确认后会发 HOME 把姿态转回来, 然后返回 False。
        """
        t0 = time.ticks_ms()
        self.turns = 0
        self.net_turn = 0
        self.lost = 0
        self.homed = False
        self.wait = 0
        # 中心先验也清掉: align() 是在做完肢体动作、或者刚回正之后调的,
        # 机器人的位置和朝向跟上一次量线那会儿已经没关系了, 留着只会认错边。
        self.c_far = None
        self.c_near = None
        while not app.need_exit():
            if time.ticks_ms() - t0 > timeout_s * 1000:
                if DEBUG:
                    print("[纠偏] 超时 %.1fs 仍未对准, e=%+.0fpx" % (timeout_s, self.last_e))
                return False
            bgr = image.image2cv(cam.read(), ensure_bgr=True, copy=False)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            cmd, info = self.step(gray)
            # draw() 现在就地画, 所以必须给它副本 —— bgr 是相机缓冲区的视图。
            if disp is not None and SHOW_LEVEL >= 1:
                disp.show(image.cv2image(self.draw(bgr.copy(), cmd, info), bgr=True))
            if cmd is None:
                continue
            if cmd == CMD_FWD:                      # 对准了 —— 这里不真的前进
                if DEBUG:
                    print("[纠偏] 已对准, e=%+.0fpx" % info["e"])
                return True
            if cmd == CMD_BLOCKED:
                if DEBUG:
                    print("[纠偏] 连转 %d 次无效, 放弃" % CORR_MAX_TURN)
                return False
            out.send_cmd(cmd, info["arg"])
            if cmd == CMD_HOME:                     # 丢线并已回正 → 不再死等
                if DEBUG:
                    print("[纠偏] 丢线, 已回正 %+d° 到原姿态" % info["arg"])
                return False
        return False                                # app 退出

    def draw(self, vis, cmd, info):
        """把纠偏状态画到画面上。

        ★ v17: 改成【就地画】, 不再 bgr.copy()。原来 main 里先 vis = bgr.copy()
          了一次, 进到这里又 copy 一次 —— 同一帧 640x480x3 复制了两遍(各 0.15ms),
          第二遍纯属多余, 因为 main 拿到返回值后就把自己那份扔了。
          调用方保证传进来的 vis 已经是副本(不能直接画在相机缓冲区上)。
        """
        yf = int(self.h * CORR_BAND)
        cv2.line(vis, (0, yf), (self.w, yf), (0, 200, 200), 1)
        cv2.line(vis, (int(self.cx), yf - 14), (int(self.cx), yf + 14), (200, 200, 200), 1)
        # 被判为"图卡"而剔除的 x 区间: 画成品红色竖带, 一眼能看出是不是误杀了真线
        for a, b in self.zones:
            cv2.rectangle(vis, (max(0, int(a)), yf - 9),
                          (min(self.w - 1, int(b)), yf + 9), (255, 0, 255), 1)
        for x, y in self.pts:
            cv2.circle(vis, (int(x), int(y)), 7, (0, 255, 255), -1)
            cv2.line(vis, (int(self.cx), int(y)), (int(x), int(y)), (0, 255, 255), 2)
        # 远带两条车道线的【预测位置】: 画成蓝色短竖线。只认落在它附近的暗段,
        # 所以在机器上一眼就能看出"那条线为什么没被认" —— 是跑出窗口了, 还是真没扫到。
        if self.c_far is not None:
            for px in (self.c_far - self.half, self.c_far + self.half):
                if 0 <= px < self.w:
                    cv2.line(vis, (int(px), yf - 12), (int(px), yf + 12), (255, 160, 0), 1)
        cv2.putText(vis, "e%+.0f tol%.0f %s %s turn%d net%+d%s%s" %
                    (info["e"], self.e_ok if self.turns > 0 else self.e_tol,
                     info["flag"] or "LOST", CMD_NAME.get(cmd, "-"), info["turns"],
                     self.net_turn, " LOST%d" % self.lost if self.lost else "",
                     "" if self.c_far is not None else " NOPRIOR"),
                    (4, yf - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if self.aligned else (0, 165, 255), 1)
        return vis


# ============================ 6. 多帧投票 + 串口 ============================
class Voter:
    """滑窗多数票 —— "识别到了就发指令"。

    不用"连续 N 帧完全一致": 边走边识别时视角一直在变, 中间插一帧误判就把
    连续性打断, 像十字形这种边界形状几乎永远凑不满连续票。
    改成最近 VOTE_WIN 次里出现 >= VOTE_NEED 次且严格领先第二名 → 立刻上报。
    低置信度的帧直接不进窗口, 不让它们稀释有效样本。

    两层"不重报", 别混淆:
      · locked —— 短期。同一张卡在视野里停留十几帧, 只认一次; 卡离开视野
        (voter.lost())就解锁。
      · sent   —— 永久(REPORT_ONCE)。某一种形状一旦【发出去过】就退役,
        lost() 不清它。环形赛道上第二次经过同一张卡时, 它负责保持沉默。
    """

    def __init__(self, win=VOTE_WIN, need=VOTE_NEED):
        self.win, self.need = win, need
        self.buf, self.locked = deque(maxlen=win), None
        self.sent = set()       # 已经成功发出去的形状 —— 永久退役, lost() 不清

    def done(self, name):
        """这一种形状是不是已经报过了(报过 = 不再发、不再打印)。"""
        return bool(REPORT_ONCE and name is not None and name in self.sent)

    def push(self, name, conf=1.0):
        if name is None or conf < VOTE_CONF:
            self.buf.append(None)                 # 占位, 让旧票照常老化出窗
            return None
        if self.done(name):
            # 已退役 → 当作一张弃权票。注意是 append(None) 而不是直接 return:
            # 让它照常占位老化, 窗口才不会被一串退役票堵住。
            # 顺带还有个好处: 退役形状的票不再参与计数, 万一新卡被偶尔误读成
            # 那个旧形状, 这些误票不会再跟正确答案抢票。
            self.buf.append(None)
            return None
        self.buf.append(name)

        cnt = {}
        for x in self.buf:
            if x is not None:
                cnt[x] = cnt.get(x, 0) + 1
        if not cnt:
            return None
        order = sorted(cnt, key=cnt.get, reverse=True)
        top = order[0]
        second = cnt[order[1]] if len(order) > 1 else 0
        if cnt[top] < self.need or cnt[top] <= second:
            return None
        if top == self.locked:                    # 同一张卡不重复上报
            return None
        self.locked = top
        self.sent.add(top)                        # 退役: 这一种以后不再报
        return top

    def lost(self):
        """图卡已离开视野 —— 清票并解锁。

        只清 locked, **不清** sent: sent 是"这一种已经报过了"的永久记录,
        清掉的话跑第二圈经过同一张卡又会重报一遍, 本版要堵的就是这个。
        """
        self.buf.clear()
        self.locked = None


class SerialOut:
    def __init__(self, port, baud):
        self.dev = None
        if not port:
            return
        try:
            self.dev = uart.UART(port, baud)
            print("[UART] %s @%d 已打开" % (port, baud))
        except Exception as e:
            print("[UART] %s 打开失败: %s —— 只输出到控制台" % (port, e))

    def send(self, name, sid):
        if SHOW_CONSOLE:
            print(">>> 识别到 %s (%s)  id=%d" % (CN_NAME[name], name, sid))
        if self.dev is None:
            return
        try:
            if SERIAL_MODE == "binary":
                self.dev.write(bytes([0xAA, 0x55, sid, sid ^ 0xA5, 0x0A]))
            else:
                self.dev.write(("SHAPE,%s,%d\n" % (name.upper(), sid)).encode())
        except Exception as e:
            print("[UART] 发送失败:", e)

    def send_cmd(self, cmd, arg):
        """发送新图像的建议，ASCII或AA57；执行及投票由STM32负责。

        TURN只表示左右方向，不把arg换算成重复转向。SHIFTL/SHIFTR执行一次。
        不发送HOME、不用固定帧等待猜测动作完成。
        """
        name = CMD_NAME.get(cmd, "HOLD")
        if SHOW_CONSOLE:
            print("[指令] %s %d" % (name, arg))
        if self.dev is None:
            return
        try:
            if SERIAL_MODE == "binary":
                a = int(max(-128, min(127, arg))) & 0xFF
                self.dev.write(bytes([0xAA, 0x57, cmd, a, (cmd ^ a) & 0xFF, 0x0A]))
            else:
                self.dev.write(("CMD,%s,%d\n" % (name, arg)).encode())
        except Exception as ex:
            print("[UART] 指令帧发送失败:", ex)


# ============================ 7. 主循环 ============================
def main():
    cam = camera.Camera(CAM_W, CAM_H)
    disp = display.Display()
    print("[CAM] %dx%d 已启动 | SHOW_LEVEL=%d DEBUG=%s"
          % (CAM_W, CAM_H, SHOW_LEVEL, DEBUG))
    print("[VISION_CFG] quad_guard=1 area=%.3f..%.2f final_quad_bounds=1px" % (
        AREA_MIN_REL, AREA_MAX_REL))
    out = SerialOut(UART_PORT, UART_BAUD)

    tpl = build_templates()               # 只给 Hu 矩兜底用; 画像是上面手写的
    voter = Voter()
    corr = LineCorrector(CAM_W, CAM_H)
    miss, clear_n, act_hold = 0, 0, 0
    quad, card_view, show_name, show_conf = None, None, None, 0.0
    cmd, cinfo = None, {"e": 0.0, "arg": 0, "flag": "", "turns": 0}
    last_cmd_tick, last_sent_cmd, frame_seq = -100000, None, 0

    while not app.need_exit():
        bgr = image.image2cv(cam.read(), ensure_bgr=True, copy=False)
        frame_seq += 1
        # 整个流水线的灰度源头只有这一处 cvtColor。往下所有需要灰度的环节
        # (快筛 / 墨迹掩码 / 透视矫正 / 形状提取 / 扫描带)都复用它, 不再各转各的。
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # ★ 顺序: 先跑图形识别, 再跑纠偏。
        #   纠偏要用本帧的图卡四边形把图卡边框从"赛道黑线"里挖掉, 所以必须
        #   等图形流水线先把卡片位置算出来。反过来排的话只能用上一帧的四边形,
        #   机器人在走, 一帧的位移就足以让挖的位置错开。

        # ---------------- 图形识别: 边走边识别 ----------------
        # 快筛每帧都跑(只碰 1/16 像素); 快筛一报疑似, 就每帧跑完整流水线。
        # 图卡在视野里只停留十几帧, 隔帧降频等于白扔识别机会。
        ran_shape = (not GATE_ON) or card_likely(gray)
        if not ran_shape:
            quad, card_view, show_name = None, None, None
            clear_n += 1
            if clear_n >= GATE_CLEAR_N:       # 视野里确实空了 → 解锁, 准备认下一张卡
                voter.lost()
                miss, clear_n = 0, 0
        else:
            clear_n = 0
            # ink_masks 现在是生成器: 第一张(Otsu)能定出四边形就不会算后面三张。
            quad, why = find_card_quad(ink_masks(bgr, gray))
            name, conf, feat, card_view = None, 0.0, None, None

            if quad is not None:
                # 识别全程走灰度矫正图; 彩色的只有画缩略图时才算(见显示段)。
                card_gray = rectify(gray, quad)
                if SHOW_LEVEL >= 2:
                    card_view = rectify(bgr, quad)   # 只为右上角缩略图, 识别不用
                conf = detect_cross_center(card_gray)
                if conf > 0:
                    name = "cross"
                    if DEBUG and not voter.done("cross"):
                        print("[十字] 中心黑点及上下左右四臂齐全 conf=%.2f" % conf)
                else:
                    cnt, _, used = extract_shape(card_gray)
                    if cnt is not None:
                        feat = features(cnt)
                        if feat is not None:
                            name, conf, scores = classify(cnt, feat, tpl)
                            if name == "cross" or conf < CONF_MIN:
                                name = None          # 无中心四臂证据，不报十字
                            # ★ 已退役的形状彻底静音 —— 连特征行都不打。
                            #   环形赛道第二圈会再次经过同一张卡, 它在视野里要停
                            #   十几帧, 每帧一行特征, 真正有用的新卡信息会被刷掉。
                            if DEBUG and not voter.done(name):
                                top = sorted(scores.items(), key=lambda kv: -kv[1])[:2]
                                print("[特征] 圆度%.2f 凸性%.2f 填率%.2f 顶点%d 凹点%d 深凹%d "
                                      "边角%s 内缩%.2f | 得分 %s=%.2f %s=%.2f -> %s %.2f"
                                      % (feat["circ"], feat["solid"], feat["fill"], feat["v"],
                                         feat["reflex"], feat["deep"],
                                         "--" if feat["edge"] is None else "%.1f" % feat["edge"],
                                         used, top[0][0], top[0][1], top[1][0], top[1][1],
                                         name, conf))
                    elif DEBUG:
                        print("[形状] 卡片已定位但 ROI 里没提到形状, 内缩到 %.2f 仍失败"
                              % INSET_RETRY[-1])
                miss = 0
            else:
                miss += 1
                if DEBUG and miss % 10 == 0:
                    print("[扫描] 没找到卡片 | %s | 太远/太斜/边框太淡?" % why)
                if miss > 25:
                    voter.lost()
                    miss = 0

            # 识别到了就发 —— 滑窗多数票, 不等连续帧
            # push() 内部会挡掉已退役的形状(REPORT_ONCE), 所以同一种形状
            # 这里最多只会 hit 一次。
            hit = voter.push(name, conf)
            if hit is not None:
                out.send(hit, SHAPE_ID[hit])
                if SHOW_CONSOLE and REPORT_ONCE:
                    print("    (已报 %d/%d: %s —— 这几种之后不再打印)"
                          % (len(voter.sent), len(SHAPES),
                             " ".join(CN_NAME[s] for s in SHAPES if s in voter.sent)))
                # ★ 发完立刻转入循线模式: 不再暂停纠偏(ACT_HOLD_N=0), 并且清掉
                #   纠偏器残留的等待节拍, 让它在**本帧**就重新测偏差、发指令。
                #   识别图卡那几帧里纠偏多半刚发过 FWD/TURN, 不清的话还要白等
                #   CORR_SETTLE 帧才动。
                act_hold = ACT_HOLD_N if UART_PORT else 0
                corr.resume()
            show_name, show_conf = name, conf

        # ---------------- 巡线: 每帧测量，限频发送新建议，STM32周期末执行 ----------------
        # 上报形状后立刻接管(ACT_HOLD_N=0), 中间不留空档 —— 图卡贴在赛道边,
        # 上报那一刻机器人往往正偏向卡那一侧, 正是最该纠偏的时候。
        # act_hold 机制保留着: 把 ACT_HOLD_N 改回 25 就恢复"等下位机做完动作"。
        # quad 传进去: 把图卡的黑边框从"赛道黑线"里挖掉。
        cmd = None
        if act_hold > 0:
            act_hold -= 1
        elif CORR_ON:
            cmd, cinfo = corr.step(gray, quad)
            if cmd is not None:
                now = time.ticks_ms()
                urgent = cmd in (CMD_HOLD, CMD_BLOCKED) and cmd != last_sent_cmd
                if urgent or now - last_cmd_tick >= CORR_SEND_INTERVAL_MS:
                    out.send_cmd(cmd, cinfo["arg"])
                    last_cmd_tick, last_sent_cmd = now, cmd
                    if SHOW_CONSOLE:
                        print("[LINE_TX] frame=%d tick=%d cmd=%s arg=%d e=%+.1f flag=%s lost=%d" % (
                            frame_seq, now, CMD_NAME[cmd], cinfo["arg"], cinfo["e"],
                            cinfo["flag"] or "LOST", corr.lost))

        # ---------------- 显示: 分级, 比赛模式整条跳过 ----------------
        # SHOW_LEVEL = 0 时这一整段连一次 copy 都不做, 更不碰 framebuffer。
        # 识别和串口在上面已经全部跑完了 —— 显示从来不参与任何判定。
        if SHOW_LEVEL >= 1:
            # 必须 copy: bgr 是 image2cv(copy=False) 拿到的相机缓冲区视图,
            # 直接往上画会污染下一帧的输入。整幅只复制这一次(原版复制两次)。
            vis = bgr.copy()

            if SHOW_LEVEL >= 2:
                # 纠偏: 远带、中线、线心连线、预测位、当前指令(就地画, 不再 copy)
                if CORR_ON:
                    corr.draw(vis, cmd, cinfo)

                # 图形: 绿框 + 右上角矫正后的卡片缩略图
                if quad is not None:
                    cv2.polylines(vis, [quad.astype(np.int32)], True, (0, 255, 0), 2)
                if card_view is not None:
                    vis[8:118, CAM_W - 118:CAM_W - 8] = cv2.resize(card_view, (110, 110))
                    cv2.rectangle(vis, (CAM_W - 119, 7), (CAM_W - 7, 119), (0, 255, 0), 1)
                txt = "%s %s" % (show_name or "--",
                                 "" if show_name is None else "%.2f" % show_conf)
                # 已退役的形状仍然照常画在屏幕上, 只是标成 DONE 并画成灰色 ——
                # 串口和控制台安静了, 但要能一眼看出"相机还看得见它, 是故意不报的",
                # 而不是识别挂了。
                shown_done = voter.done(show_name)
                if shown_done:
                    txt += " DONE"
                cv2.putText(vis, txt, (4, CAM_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (160, 160, 160) if shown_done else
                            ((0, 255, 0) if show_name else (0, 165, 255)), 2)
                # 右下角列出已报过的形状(按串口 ID, 1~6), 跑完一圈回来好对账。
                # 用 ID 不用首字母: circle/cross 都是 C、star/square 都是 S, 会撞。
                if REPORT_ONCE and voter.sent:
                    cv2.putText(vis, "SENT " + "".join(str(SHAPE_ID[s]) for s in SHAPES
                                                       if s in voter.sent),
                                (CAM_W - 150, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (160, 160, 160), 1)
                if act_hold > 0:
                    cv2.putText(vis, "ACT %d" % act_hold, (CAM_W - 110, CAM_H - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.putText(vis, "%.1ffps" % time.fps(), (4, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 0), 1)
            disp.show(image.cv2image(vis, bgr=True))
        else:
            time.fps()      # 比赛模式也要喂一下帧率计数器, 不然 time.fps() 失准


if __name__ == "__main__":
    main()
