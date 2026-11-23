"""주행 도구 공통 안전 종료.

어떤 경로로 끝나든(정상 종료, 예외, Ctrl+C, 타임아웃 SIGTERM)
    1) /cmd_vel 에 0 을 여러 번 발행하고
    2) /base_node/release 로 축을 해제한다.

축을 켜 둔 채 스크립트가 끝나면 로봇이 언제든 다시 움직일 수 있는
상태로 남는다. 실제로 그 상태로 방치한 적이 있어 공통화했다.

사용:
    from safe_stop import install_safe_stop
    install_safe_stop(node)          # node 는 /cmd_vel 발행자를 가진 rclpy Node
"""
import atexit
import signal
import time

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

_done = False


def stop_now(node, release=True, pubs=15):
    """즉시 정지 + 축 해제. 여러 번 불려도 안전하다."""
    global _done
    try:
        pub = getattr(node, "pub", None) or getattr(node, "pub_cmd", None)
        if pub is not None:
            for _ in range(pubs):
                pub.publish(Twist())
                try:
                    import rclpy
                    rclpy.spin_once(node, timeout_sec=0.02)
                except Exception:
                    time.sleep(0.02)
        if release:
            cli = node.create_client(Trigger, "/base_node/release")
            if cli.wait_for_service(timeout_sec=2.0):
                fut = cli.call_async(Trigger.Request())
                t0 = time.time()
                import rclpy
                while not fut.done() and time.time() - t0 < 3.0:
                    rclpy.spin_once(node, timeout_sec=0.05)
                node.get_logger().info("정지 및 축 해제 완료")
    except Exception as e:
        try:
            node.get_logger().warn("안전 정지 중 오류: %s" % e)
        except Exception:
            pass
    _done = True


def install_safe_stop(node, release=True):
    """종료 경로 전부에 안전 정지를 건다.

    rclpy.shutdown() 이 먼저 돌면 발행자 컨텍스트가 죽어 정지 지령이 못 나간다.
    실제로 그 순서 때문에 축이 켜진 채 끝난 적이 있다. 그래서 shutdown 을
    가로채 그 직전에 먼저 정지시킨다.
    """
    import rclpy

    if not getattr(rclpy, "_safe_stop_wrapped", False):
        _orig_shutdown = rclpy.shutdown

        def _shutdown(*a, **kw):
            if not _done:
                try:
                    stop_now(node, release)
                except Exception:
                    pass
            return _orig_shutdown(*a, **kw)

        rclpy.shutdown = _shutdown
        rclpy._safe_stop_wrapped = True

    atexit.register(lambda: (None if _done else stop_now(node, release)))

    def _sig(signum, frame):
        stop_now(node, release)
        raise SystemExit(0)

    for s in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(s, _sig)
        except Exception:
            pass
