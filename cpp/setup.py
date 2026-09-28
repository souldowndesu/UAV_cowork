# -*- coding: utf-8 -*-
"""构建 C++ 加速扩展 `_fast`（pybind11）。用 `build_ext --inplace` 在 cpp/ 下编译，
再由 build_extension.bat 拷贝 `_fast*.pyd` 到 nav/ 目录。"""
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

ext_modules = [
    Pybind11Extension(
        "_fast",
        ["fast_kernels.cpp", "fast_planning.cpp"],
        extra_compile_args=["/O2", "/EHsc", "/utf-8"],  # Windows MSVC
    ),
]

setup(
    name="_fast",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
