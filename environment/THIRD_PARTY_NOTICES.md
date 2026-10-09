# Bundled runtime dependencies

The offline `wheelhouse` preserves third-party packages separately from NavigationPhase1 source. Package versions are locked in `requirements.txt`.

- AirSim 1.8.1: Copyright (c) Microsoft Corporation; MIT License. Full notice: `licenses/AirSim-LICENSE.txt`. Source: https://github.com/microsoft/AirSim/tree/v1.8.1
- msgpack-rpc-python 0.4.1: Copyright (c) 2011- Masahiro Nakagawa; Apache License, Version 2.0. Full license: `licenses/Apache-2.0.txt`. Upstream copyright and license declaration: https://github.com/msgpack-rpc/msgpack-rpc-python#copyright
- msgpack-python 0.5.6: its COPYING notice remains inside the wheel.
- NumPy 1.26.3, SciPy 1.15.3, OpenCV contrib Python 4.11.0.86 and Tornado 4.5.3: their license texts and bundled component notices remain inside their wheels and are installed with their packages.

The AirSim, msgpack-rpc-python, msgpack-python and Tornado wheels were built from the upstream package source distributions for the verified CPython 3.10 Windows environment. They are provided as installation artifacts, with original Python package code retained. Compiled extensions unavailable during dependency wheel construction use their upstream Python fallback; this combination passed RPC and simulator validation.
