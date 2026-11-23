"""시험 준비.

cart_match 는 ROS 의존이 없어 그냥 불러온다.
api_node 는 rclpy 를 import 하지만, 시험하는 것은 계산 함수뿐이라
모듈만 올리고 노드는 만들지 않는다. 인스턴스가 필요한 함수는
__new__ 로 껍데기만 만들어 필요한 값만 채워 쓴다.
"""
import importlib.util
import pathlib
import sys

import pytest

SRC = pathlib.Path(__file__).resolve().parent.parent / "und_amr_bringup"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, SRC / (name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="session")
def cm():
    return _load("cart_match")


@pytest.fixture(scope="session")
def api():
    return _load("api_node")


@pytest.fixture(scope="session")
def ApiNode(api):
    for v in vars(api).values():
        if isinstance(v, type) and hasattr(v, "_compose_map"):
            return v
    raise RuntimeError("ApiNode 클래스를 찾지 못했습니다")
