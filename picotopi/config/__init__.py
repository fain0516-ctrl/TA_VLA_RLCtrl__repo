# -*- coding: utf-8 -*-
"""설정값 단일 출처. `from picotopi import config as C` 로 V8 `v8.config`와 같은 이름을 씁니다.

표기: [가이드 §x] docs/research/04_team_guide.md 해당 절, [추정] 실측 전 임시값.
"""
from .timing import *      # noqa: F401,F403
from .mechanics import *   # noqa: F401,F403
from .sensing import *     # noqa: F401,F403
from .control import *     # noqa: F401,F403
from .task import *        # noqa: F401,F403
from .dr import *          # noqa: F401,F403
from .mechanics import tendon_force_max   # noqa: F401
from .dr import dr_bounds, dr_sample, adr_sides   # noqa: F401
