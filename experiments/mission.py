"""Shared mission execution and streaming recording used by scene_test."""
from __future__ import annotations

import os
import time

import numpy as np

from navigation.pipeline import NavigationPipeline


def _sense_bbox(pipe, cfg):
    """感知区（A* 计算域）bbox = 轨迹包围盒 + 局部图半宽膨胀（含前向偏置）。

    持久图只存 FREE/OCCUPIED，未记录即 unknown；只有落在感知区内的 unknown 才有观察
    意义（用户 m03748）。这里用轨迹包围盒向外扩局部图半宽近似感知区并集（x 向前向余量
    更大，map_back_x）；回放端还可再手动裁剪（persistent_bbox）。
    """
    tl = pipe.telemetry()
    if not tl:
        return None
    xs = [float(t["x"]) for t in tl]
    ys = [float(t["y"]) for t in tl]
    zs = [float(t["z"]) for t in tl]
    ms = cfg.map_size
    back_x = float(getattr(cfg, "map_back_x", ms[0] / 2.0))
    return (min(xs) - back_x, max(xs) + (ms[0] - back_x),
            min(ys) - ms[1] / 2.0, max(ys) + ms[1] / 2.0,
            min(zs) - ms[2] / 2.0, max(zs) + ms[2] / 2.0)


def run_mission(io, cfg, goal, on_tick=None,
                recorder=None, viz_server=None,
                stuck_timeout=30.0, stuck_dist=2.0,
                autosave_path=None, autosave_interval=5.0,
                max_duration=0.0):
    """跑一个完整任务，返回 (reached, telemetry, duration, outcome)。

    ``recorder`` 非空时完整录制；``viz_server`` 非空时启动可视化服务（需 recorder）。
    ``outcome`` 包括 reached / safe_approach / collided / stuck / timed_out / interrupted / error。
    卡住 = 连续 ``stuck_timeout`` 秒内
    位置位移 < ``stuck_dist`` 米（无人机顶墙/原地振荡，不再前进）。
    ``autosave_path`` 非空时，每 ``autosave_interval`` 秒把录制落盘一次（防进程被
    硬杀后丢失全部数据）；录像以 v2 分块追加，地图快照按此周期刷新。
    ``max_duration`` > 0 限制导航阶段墙钟时长，不包含起飞/爬升；0 不限时。
    """
    io.arm()
    io.takeoff()
    time.sleep(1.0)
    io.move_to_z(-cfg.flight_altitude, 2.0)

    pipeline_class = NavigationPipeline
    if cfg.navigation_mode == 'safe_route':
        from navigation.safe_navigation import SafeNavigationPipeline
        pipeline_class = SafeNavigationPipeline
    pipe = pipeline_class(io, cfg)
    pipe.set_goal(goal)
    if recorder is not None:
        pipe.recorder = recorder
        recorder.set_goal(goal)
        if autosave_path:
            recorder.start(autosave_path,cfg.to_dict())
    if viz_server is not None:
        viz_server.recorder = recorder
        viz_server.start()
    pipe.start()

    t0 = time.time()
    outcome = "error"
    last_progress_pos = None
    last_progress_t = time.time()
    last_autosave_t = t0
    try:
        while True:
            time.sleep(0.2)
            if on_tick:
                on_tick(pipe)
            # 周期性自动落盘：保证进程被硬杀/崩溃时磁盘上仍有最近的完整录制
            if recorder is not None and autosave_path and \
                    time.time() - last_autosave_t >= autosave_interval:
                try:
                    # 每次自动落盘前刷新持久图快照：否则 persistent 只在 mission 完全
                    # 结束（pipe.stop 后）才写入，进程被硬杀 / final save 中断时录制里
                    # 就只剩空的 persistent（用户 m03891 问题1：replay 没显示所有
                    # occupied）。周期刷新保证 autosave 的 recording 始终含最近快照。
                    if getattr(pipe, "persistent", None) is not None:
                        recorder.set_persistent(
                            pipe.persistent.occupied_points(stride=2),
                            unknown=None,
                            bbox=_sense_bbox(pipe, cfg))
                    d = os.path.dirname(autosave_path)
                    if d:
                        os.makedirs(d, exist_ok=True)
                    recorder.save(autosave_path)
                except Exception as e:
                    print(f"[Mission] 自动落盘失败：{e}")
                last_autosave_t = time.time()
            if pipe.reached:
                outcome = "safe_approach" if getattr(pipe,'reached_safe',False) else "reached"
                print(f"[Mission] 到达目标 {goal} (耗时 {time.time()-t0:.1f}s)")
                break
            if pipe.collided:
                outcome = "collided"
                print("[Mission] 检测到碰撞，中止")
                break
            if max_duration > 0 and time.time() - t0 >= max_duration:
                outcome = "timed_out"
                print(f"[Mission] 达到测试时限 {max_duration:.0f}s，中止")
                break
            # 卡住检测：基于最近遥测位置，长时间无显著前进则中止（避免无限顶墙且保证能保存数据）
            tl = pipe.telemetry()
            if tl:
                last = tl[-1]
                pos = np.array([last["x"], last["y"], last["z"]], dtype="float64")
                if last_progress_pos is None or \
                        np.linalg.norm(pos - last_progress_pos) >= stuck_dist:
                    last_progress_pos = pos
                    last_progress_t = time.time()
                elif stuck_timeout > 0 and time.time() - last_progress_t > stuck_timeout:
                    outcome = "stuck"
                    print(f"[Mission] 卡住（{stuck_timeout:.0f}s 内位移 < {stuck_dist}m），中止")
                    break
    except KeyboardInterrupt:
        outcome = 'interrupted'
        print('[Mission] Ctrl+C 中断，保存部分遥测和录像')
    except Exception as exc:
        outcome = 'error'
        print(f'[Mission] 导航异常：{exc}')
        import traceback
        traceback.print_exc()
    finally:
        try:
            pipe.stop()
        finally:
            try:
                if recorder is not None:
                    if getattr(pipe, 'persistent', None) is not None:
                        recorder.set_persistent(pipe.persistent.occupied_points(stride=2),
                                                unknown=None, bbox=_sense_bbox(pipe,cfg))
                    recorder.record_event('outcome',dict(t=time.time(),outcome=outcome))
            except Exception as exc:
                print(f'[Mission] 录像最终快照失败：{exc}')
            finally:
                try:
                    if recorder is not None:recorder.close()
                except Exception as exc:
                    print(f'[Mission] 录像收尾失败：{exc}')
                finally:
                    try:io.land()
                    except Exception:pass
    return outcome in ('reached','safe_approach'), pipe.telemetry(), time.time() - t0, outcome

