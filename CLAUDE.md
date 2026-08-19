# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

---

# 프로젝트 인계 노트 — 대시보드 버그수정 · 세분화 · DPI 이슈

> 직전 세션(웹 채팅, 로컬 파일 직접 접근 불가)에서 진행한 내용의 인계 문서.
> 이 프로젝트는 재난구조 로봇 시뮬레이션/대시보드로, 주요 파일은
> `dashboard.py`, `map_canvas.py`, `sim_bridge.py`, `rescue_planner.py`,
> `robot_config.py`. `path_planner.py`/`path_planner_sim2.py`는 해당
> 세션에서 건드리지 않았다.

## 0. 해당 세션에서 한 것 — 한눈에

1. `map_canvas.py`: 범례 "로봇" 항목이 사라지는 버그 발견·수정
2. `robot_config.py` + 3파일: 위험 임계값(`FIRE_TEMP_C`/`GAS_PPM`) 단일 출처화
3. `sim_bridge.py`: FIRE 위험구역 원이 과도하게 크게 그려지던 문제 수정 (클러스터링 도입)
4. `dashboard.py`: 히트맵(`HeatGrid`) 셀이 직사각형으로 늘어나던 문제 수정
5. `dashboard.py`: 히트맵 해상도 세분화(`GRID_N` 12→20) + 폰트 자동조절/생략
6. `dashboard.py`: 레이아웃 비율 조정 (카메라:히트맵:하단 = 3:3:2 → 2:5:2)
7. `dashboard.py`: 히트맵 폰트가 특정 환경(Windows, 고배율 DPI 추정)에서 전혀 안 보이는 문제 — `setPointSize` → `setPixelSize` 전환으로 대응 (★미확정 — 사용자 재확인 대기 중, §6 참고)

## 1. `map_canvas.py` — 범례 "로봇" 항목 사라짐 버그

**증상**: 대시보드 지도 상단 범례에 10개 항목(자유공간~로봇) 중 마지막 "로봇" 항목이 화면에 아예 안 보임.

**원인**: `_compute_legend_layout()`(줄바꿈 계산)은 범례 시작점을 `pad_l`(46px)로 가정하고 계산하는데, `_draw_legend()`(실제 그리기)는 지도 중심정렬 좌표 `ox`에서 그리기 시작한다. 지도가 위젯 높이에 맞춰 폭보다 좁게 그려지면(세로 제약) `ox`가 `pad_l`보다 훨씬 커지고(실측: 46 vs 193, 147px 차이), 계산할 때 가정한 폭 안에 들어간다고 판단한 마지막 항목이 실제로 그릴 때는 위젯 경계 밖으로 밀려 나갔다. Qt는 위젯 밖으로 그려도 에러 없이 그냥 안 보이게 처리하므로 조용히 사라진 것처럼 보였다.

지도가 정사각형이라 좌우 여백이 남는 특정 창 비율에서 재발하는 케이스였다.

**수정**: `_draw_legend()`의 시작점을 `ox` → `pad_l`로 통일.

```python
def _draw_legend(self, p):
    s_, ox, oy, n = self._tf
    pad_l = 46 if self.show_axes else 8      # ← 추가
    f = QFont(); f.setPointSize(7); p.setFont(f)
    top = oy - self.LEGEND_H + self.LEGEND_PAD_TOP
    for row_i, row in enumerate(self._legend_rows):
        y = top + row_i * self.LEGEND_ROW_H
        x = pad_l                             # ← ox 였던 걸 pad_l 로
```

**검증**: 오프스크린 렌더링으로 수정 전/후 스크린샷 비교, "로봇" 항목 정상 표시 확인.

## 2. 위험 임계값 단일 출처화

**배경**: `FIRE_TEMP_C=50`, `GAS_PPM=40`이 `sim_bridge.py`, `rescue_planner.py`, `dashboard.py` 세 파일에 따로 박혀 있었다. 특히 `sim_bridge.py`의 가스 임계값은 이름도 없이 그냥 숫자 `40`이 두 번 하드코딩돼 있었고, `dashboard.py`의 가스 카드 "위험" 기준은 아예 다른 값(`60`)이었다.

**수정**: `robot_config.py`에 4개 상수 추가.

```python
# robot_config.py
FIRE_TEMP_C = 50.0      # 이 온도(°C) 이상 = 화재 위험 (지도표시+경로회피+카드위험)
GAS_PPM = 40.0          # 이 농도(ppm) 이상 = 가스 위험 (지도표시+경로회피+카드위험)
TEMP_WARN_C = 40.0      # 카드 "경계(주의)" 조기경고선
GAS_WARN_PPM = 30.0
```

`sim_bridge.py`, `rescue_planner.py`, `dashboard.py` 세 곳은 모두 동일한 방어 패턴으로 가져온다 — `robot_config.py`가 없어도(시뮬 단독 실행 워크플로우) 안 깨지도록 `getattr` + 기본값 fallback:

```python
try:
    import robot_config as _RC   # dashboard.py 는 기존 C 변수 재사용
except ImportError:
    _RC = None
FIRE_TEMP_C  = getattr(_RC, "FIRE_TEMP_C", 50.0)
GAS_PPM      = getattr(_RC, "GAS_PPM", 40.0)
```

`sim_bridge.py`의 `DashViz.FIRE_CIRCLE_TEMP_C = 50.0`(하드코딩) → `= FIRE_TEMP_C`(참조)로 변경. `_hazards()`의 이름 없던 `40` 두 곳도 `GAS_PPM`으로 교체.

**★동작 변화 — 반드시 인지할 것**: `dashboard.py` 가스 카드의 "위험" 기준이 `60` → `40`으로 내려간다(온도는 기존 `50` 그대로, 변화 없음). 카드에서 "위험"이 이제 더 낮은 농도부터 뜬다 — 지도/경로회피 기준과 일치시킨 의도된 변화.

**적용 상태 (해당 세션 종료 시점)**: diff로만 전달됨. **실제 파일 반영 여부 미확인** 상태였음 — 이번 세션에서 로컬 파일을 직접 대조해 확인 필요.

## 3. `sim_bridge.py` — FIRE 위험구역 원이 과도하게 크던 문제

**증상**: seed=13 실행 화면에서 FIRE 원이 지도 절반 가까이 덮는 걸 발견.

**원인**: `_hazards()`의 FIRE 처리가 임계값 넘는 칸 **전체의 평균**을 중심으로, **가장 먼 칸까지**를 반경으로 잡는 방식이었다. `ScalarField`가 온도 발생원을 2개 두는데, 로봇이 둘 다 밟으면 두 발생원 사이 안 뜨거운 지대까지 원 안에 포함돼 시각적으로 과장됐다.

**배경**: "화재는 여러 개 말고 원 하나만 그려달라"는 게 이전 세션의 명시적 요청이었어서, gas는 이미 클러스터링해서 여러 개 원을 허용하는데 fire는 클러스터링 없이 무조건 하나로 뭉쳐 그리게 짜여 있었다.

**수정**: "원 하나" 원칙은 유지하되, 핫셀을 union-find로 클러스터링(가스 병합거리와 동일 9칸 기준)해서 서로 멀리 떨어진 발생원은 별개 그룹으로 나누고, **가장 지배적인(셀 개수 많은) 덩어리 하나만** 원으로 그린다.

```python
def _cluster_points(points, gap):
    """points: [(x, y, v), ...]. gap 칸 이내로 연결된(전이적으로도) 점들을
    한 그룹으로 묶는다(union-find)."""
    n = len(points)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        for j in range(i + 1, n):
            if math.hypot(points[i][0] - points[j][0],
                         points[i][1] - points[j][1]) < gap:
                ri, rj = find(i), find(j)
                if ri != rj:
                    parent[ri] = rj

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(points[i])
    return list(groups.values())
```

```python
# _hazards() 안 fire 처리부
fire_cells = [(x, y, v) for (x, y), v in self.measured_t.items()
             if v >= self.FIRE_CIRCLE_TEMP_C]
if fire_cells:
    dominant = max(_cluster_points(fire_cells, gap=9), key=len)
    cx = sum(x for x, y, v in dominant) / len(dominant)
    cy = sum(y for x, y, v in dominant) / len(dominant)
    r = max(math.hypot(x - cx, y - cy) for x, y, v in dominant) + 1.5
    level = min(max((v - self.FIRE_CIRCLE_TEMP_C) / 20
                    for x, y, v in dominant), 1.0)
    merged.append({"kind": "fire", "x": cx, "y": cy,
                  "r": max(r, 3.0), "level": level})
```

**★트레이드오프**: 두 번째 발생원이 실제로 위험해도 관측 셀이 적으면(로봇이 아직 덜 밟았으면) 아예 원이 안 그려질 수 있다 — "원 하나" 원칙상 감수하는 부분. 여러 개 원을 허용하는 쪽으로 바꾸고 싶어지면 이 함수의 `max(..., key=len)` 한 줄만 걷어내면 gas처럼 여러 원이 나오게 바뀐다.

**검증**: 6칸짜리 핫존 + 62칸 떨어진 2칸짜리 핫존으로 합성 테스트 → 큰 쪽만 선택되고 반경 3.0으로 정확히 그 근처만 감쌈 확인. seed=13 재생으로 실제 개선 확인.

**적용 상태 (해당 세션 종료 시점)**: diff로만 전달됨. **실제 파일 반영 여부 미확인** 상태였음.

## 4. `dashboard.py` — `HeatGrid` 셀이 직사각형으로 늘어나던 문제

**증상**: 온도맵/가스맵 카드가 지도(정사각형)와 달리 셀이 직사각형으로 찌그러져 보임.

**원인**: `HeatGrid.paintEvent`가 `cw, ch = W/n, H/n`로 폭/높이를 따로 계산해서, 카드가 정사각형이 아니면 셀도 직사각형이 됨.

**수정**: `MapCanvas`와 동일한 패턴 — 폭/높이 중 작은 쪽에 맞춰 정사각형 격자를 만들고, 남는 공간은 가운데 정렬 여백으로 처리.

```python
W, H = self.width(), self.height() - 14
s = min(W / n, H / n)              # 정사각형 셀 - 작은 쪽 기준
grid_px = s * n
ox = (W - grid_px) / 2              # 가운데 정렬 여백
oy = (H - grid_px) / 2
cw = ch = s
```

셀 그리는 루프의 `x, y = c * cw, r * ch` → `x, y = ox + c * cw, oy + r * ch`. 컬러바는 원래대로 위젯 전체 폭 그대로 사용(격자 위치와 무관하게 항상 같은 자리).

**적용 상태**: 통짜 파일로 전달됨, 반영 확실.

## 5. `dashboard.py` — 히트맵 해상도 세분화 (`GRID_N` 12 → 20)

**배경**: 히트맵을 더 세분화해서 보고 싶다는 요청.

**숨겨진 원인**: `dashboard.GRID_N`을 바꿔도 실제 화면엔 반영이 안 됐다 — `SimSource._start_sim2()`가 `sim_bridge.make_dash_viz()`를 호출할 때 `grid_n` 인자를 안 넘겨서, `sim_bridge.py`에 따로 박힌 기본값(12)을 쓰고 있었기 때문. `dashboard.GRID_N`과 `sim_bridge`의 `grid_n` 기본값이 "우연히 같은 숫자"였을 뿐 서로 참조하는 관계가 아니었다 — §2 임계값 문제와 같은 계열의 "따로 노는 상수" 버그.

```python
# 수정
S2._Viz = B.make_dash_viz(self.s, grid_n=GRID_N)   # grid_n 명시적으로 전달
```

**폰트 오버플로우 문제**: `GRID_N=20`으로 처음 올렸을 때 고정 폰트(7pt)가 좁아진 셀 폭을 넘어 옆 칸까지 번지는 문제 발생(숫자들이 다닥다닥 붙어 안 읽힘). → 셀 크기에 비례해 폰트 자동 축소 + 너무 좁으면 숫자 생략, 색상만 표시:

```python
px = max(7, min(13, int(s * 0.55)))
f = QFont(); f.setPixelSize(px); p.setFont(f)
show_text = s >= 8
```

(뒤에 `if show_text:`로 숫자 그리는 부분 감싸기.)

**기준값 튜닝 히스토리** (참고용, 실기준은 최종값만 유효):
- 처음 `GRID_N=30` 시도 → 완전 색상전용(숫자 없음), "폰트도 보고 싶다"는 요청으로 되돌림
- `GRID_N=15`로는 숫자 다 보임 확인, 이후 `GRID_N=20`으로 재도전
- `GRID_N=20`에서 처음엔 `show_text = s>=11` 기준으로 숫자가 안 뜸(`s=10.0`으로 딱 1px 모자람 — 카드가 폭보다 세로가 좁아서 세로 기준으로 셀 크기가 결정된 케이스) → 기준을 `s>=8`로 낮추고 최소 폰트도 낮춰서 해결

**최종값**: `GRID_N = 20`

**적용 상태**: 통짜 파일로 전달됨, 반영 확실.

## 6. `dashboard.py` — 폰트 완전 미표시 문제 (★미확정, 재확인 필요)

**증상**: 전체화면(2880x1800 해상도, Windows)에서 실행했을 때 히트맵 셀 색상은 정상 표시되는데 **숫자가 하나도 안 보임**. 실측으로 셀 크기를 픽셀 단위로 측정해보니 약 24px — `show_text` 기준(8px)을 넉넉히 넘는데도 숫자가 완전히 안 보였다.

**시도한 원인 진단**:
1. 처음엔 셀 크기 부족(threshold 문제)으로 의심 → 재측정 결과 아님(24px면 어떤 기준으로도 통과)
2. 두 번째 가설: `QFont.setPointSize`는 시스템 DPI 배율에 따라 실제 렌더링 픽셀 크기가 달라진다 — 고배율 디스플레이 환경에서 4~7pt가 사실상 0px에 가깝게 그려질 수 있음. `setPointSize` → `setPixelSize`로 전환(DPI와 무관하게 항상 지정한 실제 픽셀 수로 그려짐)

```python
px = max(7, min(13, int(s * 0.55)))
f = QFont(); f.setPixelSize(px); p.setFont(f)
...
f.setPixelSize(11); p.setFont(f)   # 컬러바 min/max 라벨도 동일하게 전환
```

**★한계 — 반드시 인지할 것**: 이 수정은 리눅스 오프스크린 환경에서는 검증했지만(정상 표시), **Windows 환경에서 실제로 문제가 해결되는지는 확인 못 했다** (직전 세션은 리눅스 샌드박스에서만 동작). 이번 세션은 Windows 로컬 환경에서 직접 동작하므로, 실제로 재실행해서 확인 가능.

**다음에 할 일**: 이 수정을 적용한 뒤에도 여전히 숫자가 안 보이면 —
1. 파이썬 실행 시 콘솔에 뜨는 에러/경고 메시지 확인
2. `pip show PySide6` 버전 확인 (특정 버전의 QPainter 폰트 렌더링 버그 가능성)
3. Windows 디스플레이 배율 설정(설정 > 디스플레이 > 배율) 확인 — 100%가 아니라면 그 값도 공유

**적용 상태**: ❌ **실제로 반영 안 됨** (2026-08-18 Claude Code로 로컬 파일 대조 확인). `dashboard.py`의 `HeatGrid.paintEvent`에는 `setPointSize`만 있고 `setPixelSize`는 파일 전체에 없다 (`f = QFont(); f.setPointSize(pt); p.setFont(f)`, 폰트 크기 공식도 `pt = max(4, min(7, int(s * 0.32)))`로 여기 적힌 것과 다름). Windows 고배율 DPI에서 히트맵 숫자가 안 보이는 문제가 아직 그대로일 가능성이 높다 — §11 참고.

## 7. `dashboard.py` — 레이아웃 비율 조정

**요청**: 온도/가스맵 카드 자리를 늘리고, 카메라영상/시스템상태/경로계획 정보는 줄이자.

**수정**: `_right()`의 세로 stretch 비율 변경.

```python
col.addWidget(c, 2)       # 카메라: 3 → 2 (실물 카메라 없인 노이즈 텍스처뿐이라 우선순위 낮음)
...
col.addLayout(grids, 5)   # 히트맵: 3 → 5 (세분화된 해상도를 살리려면 더 큰 자리 필요)
...
col.addLayout(bottom, 2)  # 하단: 2 그대로
```

`시스템 상태`/`경로 계획 정보` 카드에 최소 높이 가드 추가 (비중이 줄어도 항목 7개가 안 찌그러지도록):

```python
c3, l3 = card("시스템 상태")
c3.setMinimumHeight(180)
...
c4, l4 = card("경로 계획 정보")
c4.setMinimumHeight(180)
```

**적용 상태**: 통짜 파일로 전달됨, 반영 확실.

## 8. 파일별 최종 상태 정리 (직전 세션 종료 시점 기준)

| 파일 | 전달 방식 | 반영 확실성 |
|---|---|---|
| `dashboard.py` | 통짜 파일 2회 전달 — §4,5,7은 반영, §6은 미반영 | ✅/❌ 확인 완료 (2026-08-18, §11 참고) |
| `map_canvas.py` | diff만 전달 (§1) | ✅ 확인 완료 — `_draw_legend()`에 `x = pad_l` 적용됨 |
| `robot_config.py` | diff만 전달 (§2) | ✅ 확인 완료 — 4개 상수 전부 존재 |
| `sim_bridge.py` | diff만 전달 (§2, §3) | ✅ 확인 완료 — fallback 패턴·클러스터링 전부 존재 |
| `rescue_planner.py` | diff만 전달 (§2) | ✅ 상수 반영됨, ⚠️ 단 fallback 패턴은 없음 (§11) |
| `path_planner.py` | 미접촉 | 이전 상태 그대로 유효 |
| `path_planner_sim2.py` | 미접촉 | 이전 상태 그대로 유효 |

**과거 최우선 작업(완료)**: 위 파일 대조는 2026-08-18 완료. 아래 §11 참고.

## 9. 검증 방법 메모 (재사용 가능한 패턴)

실물 하드웨어/디스플레이 없이 Qt UI를 실제로 렌더링해서 스크린샷으로 검증하는 절차.

```bash
pip install PySide6 opencv-python-headless numpy pyqtgraph --break-system-packages

# 실행 스크립트 예시
QT_QPA_PLATFORM=offscreen python3 -c "
import sys, time
from PySide6.QtWidgets import QApplication
import dashboard as D

app = QApplication(sys.argv)
state = D.RobotState()
src = D.SimSource(state, seed=13, live_cam=False)
win = D.Dashboard(state, src)
win.resize(1440, 900)
win.show()
for _ in range(300):
    app.processEvents(); time.sleep(0.05)
win.grab().save('shot.png')
"
```

- `QT_QPA_PLATFORM=offscreen`이 핵심 — 디스플레이 없는 환경에서도 Qt가 프레임버퍼에 실제로 그린다.
- `win.resize(W, H)`로 임의 해상도 재현 가능 (전체화면 상황 재현 등).
- `QWidget.grab()`으로 픽셀 단위 스크린샷 확보 → PIL로 크롭/분석해서 버그를 "말로 추측"이 아니라 "픽셀로 측정"해서 확인할 수 있다.
- Windows 로컬 환경에서는 오프스크린 없이 실제 화면으로 직접 검증 가능 — 그쪽을 우선할 것.

## 10. 이월 항목 (다른 문서에서 넘어온 것, 여전히 유효)

- **카메라 렌즈 교체 검토**: Camera Module 3 Wide → 표준형(비와이드). `FOCAL_PX`(281)/`CAMERA_HFOV` 재보정 필요.
- **오도메트리 실측**(`MM_PER_TICK`) — 실기 배포 최대 블로커
- **§2 임계값 단일출처화**: 직전 세션에서 코드 작성은 완료, 파일 반영만 미확인
- **`main.py` 통합 시 CSV → `queue.Queue` 전환** — 아직 미착수

## 11. 검증 결과 및 다음 작업 (Claude Code, 2026-08-18)

인계 문서(§8)의 "미확인" 4개 파일을 Claude Code로 로컬 파일을 직접 열어
대조했다. 결과와 새로 발견한 이슈:

**확인됨**: `map_canvas.py`(§1), `robot_config.py`(§2), `sim_bridge.py`
(§2+§3) 세 파일은 diff 내용이 그대로 반영돼 있었다.

**★새 이슈 1 — §6 DPI 폰트 수정이 실제로 반영 안 됨**: 인계 문서는
"통짜 파일로 전달됨, 반영 확실하나 효과 미확인"이라 적었지만, 실제
`dashboard.py`의 `HeatGrid.paintEvent`에는 `setPointSize`만 있고
`setPixelSize`는 파일 어디에도 없다. 폰트 크기 공식도
`pt = max(4, min(7, int(s * 0.32)))`로, 인계 문서가 설명한
`px = max(7, min(13, int(s * 0.55)))`와 다르다 — 즉 "반영됐는데 효과가
불확실한" 게 아니라 애초에 반영 자체가 안 된 상태다. Windows 고배율
DPI 환경에서 히트맵 숫자가 안 보이는 문제가 아직 그대로일 가능성이
높다. **다음 작업**: `setPointSize` → `setPixelSize` 전환을 실제로
적용하고, Windows 로컬 환경(오프스크린 아님)에서 직접 확인할 것.

**★새 이슈 2 — `rescue_planner.py`에 fallback 패턴이 없음**: 인계
문서는 "`sim_bridge.py`/`rescue_planner.py`/`dashboard.py` 세 곳 모두
`robot_config.py`가 없어도 안 깨지도록 getattr+fallback을 쓴다"고
적었지만, 실제 `rescue_planner.py`는 45번째 줄에서
`from robot_config import FIRE_TEMP_C, GAS_PPM`로 직접 import한다.
`robot_config.py`가 없는 환경(시뮬 단독 실행 워크플로우)에서는 여기서
`ImportError`가 나서 죽는다 — 문서가 주장한 안전장치가 이 파일엔
없다. **다음 작업**: 실제로 시뮬 단독 실행이 필요한 워크플로우인지
확인 후, 필요하면 다른 두 파일과 같은 방어 패턴 추가할 것.

**참고(확인만, 조치 불필요)**: 현재 `dashboard.py`에는 커밋 안 된
로컬 수정이 있고, 그 안에서 `SimSource`의 시뮬 시드 기본값이 `13`→
`11`로 바뀌어 있다 — 인계 문서엔 없는 변경이라 의도한 것인지는
사용자 확인 필요.

**업데이트(2026-08-19, Claude Code)**: 위 §11의 두 "다음 작업" 모두
이후 세션에서 처리 완료됨 — ①`setPointSize`→`setPixelSize` 전환
적용됨(`dashboard.py`의 `HeatGrid.paintEvent`). ②`rescue_planner.py`도
`sim_bridge.py`와 동일한 `getattr`+fallback 패턴으로 교체됨(더는
`robot_config.py` 없이 못 죽는 직접 import가 아님). 이 문서의 §6/§11
"다음 작업" 문구는 낡은 정보이니 참고만 할 것 — 실제 파일이 최신
진실이다.

---

# 통신 구조 (실물 RC카 배포용)

> 이 프로젝트는 시뮬레이션뿐 아니라 실제 RC카에 탑재된다. 통신 3주체는
> 노트북(대시보드 실행) / 라즈베리파이(카메라) / ESP32(모터+ToF+온도·가스
> 센서)다. 2026-08-19 세션에서 실제 배포 구조를 점검하며 정리함.

## 토폴로지

**라즈베리파이가 WiFi 핫스팟을 쏘고, 노트북과 ESP32가 거기 클라이언트로
붙는다** (예: SSID `"pjh"` — ESP32 테스트 스케치 실측 확인, 아래 참고).
이건 네트워크 인프라(L2/L3) 레벨의 중심일 뿐, 애플리케이션 레벨 중계가
아니다 — 실제 데이터는:

- **카메라**: 라즈베리파이를 거친다. Pi가 `rpicam-vid`로 캡처해서 TCP
  서버로 스트림하고(포트 8888), 노트북이 `cv2.VideoCapture`로 클라이언트
  접속한다(`dashboard.py`/`camera_test_opencv.py`).
- **모터 명령 / ToF·온도·가스 센서 스트림**: 라즈베리파이를 **거치지
  않고** 노트북↔ESP32가 직접 소켓 통신한다(P2P). 모터 명령은 노트북→ESP32
  (`motor_commander.py`), 센서 스트림은 ESP32→노트북(포트 9999) 방향.

즉 "라즈베리파이 중심"은 핫스팟 인프라 얘기고, 제어/센서 데이터는
노트북과 ESP32가 같은 핫스팟에 붙은 두 클라이언트로서 서로 직접
이야기해야 한다는 뜻이다 — 이 전제가 깨지면(아래 "클라이언트 격리"
참고) 카메라는 멀쩡한데 로봇 제어/센서만 조용히 안 되는 헷갈리는
증상이 난다.

## 포트 맵 (`robot_config.py`)

| 포트 | 방향 | 역할 |
|---|---|---|
| 8890 (`CMD_PORT`) | 노트북 → ESP32 (ESP32가 서버) | 모터 명령 |
| 9999 (`STREAM_PORT`) | ESP32 → 노트북 (노트북이 서버) | ToF/온도/가스 센서 스트림 |
| 8888 (`CAMERA_PORT`) | 라즈베리파이 → 노트북 (Pi가 서버) | 카메라 영상 |

## ★IP 설정 — 가장 걸리기 쉬운 지점

`robot_config.py`의 `PC_IP`/`PI_IP`/`ESP32_IP` 세 값이 전부
`192.168.137.x` 대역이다. 이건 **Windows 모바일 핫스팟(ICS)의 고정
관례 주소**다 — 즉 이 값들은 원래 "**노트북이** 핫스팟을 쏘던" 예전
구조를 가정하고 잡힌 것이다. ESP32용 테스트 스케치(`.ino`, 이 저장소
밖에 있음)에서 실제로 이렇게 확인됨:

```cpp
const char* WIFI_SSID = "pjh";               // 라즈베리파이 핫스팟 SSID
const char* SERVER_IP = "192.168.137.1";     // 노트북. Pi로 테스트하려면 "192.168.137.52"
const uint16_t SERVER_PORT = 9999;
```

`SERVER_IP`가 정확히 `robot_config.PC_IP`(노트북 후보) /
`robot_config.PI_IP`(Pi 후보) 둘 다와 일치한다 — 즉 **파이썬 쪽
`robot_config.py`와 ESP32 `.ino` 펌웨어에 같은 IP값이 사람이 손으로
복사해서 이중으로 박혀 있고, 자동으로 동기화되지 않는다.**

★★**핵심 리스크**: 라즈베리파이가 핫스팟(SSID `"pjh"`)을 쏘는 지금,
그 핫스팟이 실제로 `192.168.137.0/24` 대역을 쓸 가능성은 낮다.
Raspberry Pi OS Bookworm 이후 기본 핫스팟 기능(NetworkManager 내장)은
보통 `10.42.0.x` 대역을 쓰고, 예전 방식(`hostapd`+`dnsmasq` 수동 설정)은
보통 `192.168.4.x`를 쓴다 — 어느 쪽이든 `192.168.137.x`가 아니다. 이
경우 `SERVER_IP`로 뭘 넣어도(`.1`이든 `.52`든) 그 주소가 애초에
네트워크에 존재하지 않아 `client.connect()`가 계속 실패하고, 시리얼
모니터엔 그냥 `[TCP] 서버 연결 실패`만 반복해서 찍힌다 — 왜 실패하는지
단서가 안 남는다.

**가장 빠른 진단법**: `.ino`의 `beginWiFi()` 연결 성공 시점에
`Serial.println(WiFi.localIP())`를 찍어서 ESP32 자신이 실제로 어느
서브넷(예: `10.42.0.x`)을 배정받았는지부터 확인할 것. 그 서브넷을 보면
`SERVER_IP`가 뭐여야 하는지(그리고 `robot_config.py`의 세 IP를 뭐로
고쳐야 하는지) 바로 나온다. 노트북 쪽도 같은 핫스팟에 붙은 뒤
`ipconfig`(Windows)로 실제 배정 IP를 확인해야 한다.

**배포 전 체크리스트**:
1. 라즈베리파이에서 핫스팟 서브넷 확인 (`ip addr show wlan0` 또는
   `nmcli connection show`).
2. 그 서브넷 기준으로 노트북·ESP32의 실제 IP를 재측정.
3. `robot_config.py`의 `PC_IP`/`PI_IP`/`ESP32_IP` 갱신.
4. ESP32 `.ino`의 `SERVER_IP`(들)도 **똑같이** 갱신 — 파이썬 쪽만
   고치면 반쪽만 고친 것이다(자동 동기화 없음, 위 참고).
5. `ESP32_IP`는 DHCP라 재부팅마다 바뀔 수 있음 — 매번 재확인 습관화.

## ★클라이언트 격리(AP/client isolation) — 두 번째로 걸리기 쉬운 지점

라즈베리파이 핫스팟 설정에 따라 **같은 AP에 붙은 클라이언트끼리(노트북
↔ ESP32) 서로 통신을 못 하게 격리**하는 옵션이 켜져 있을 수 있다(보안
목적으로 기본 켜짐인 핫스팟 소프트웨어도 있다). 이게 켜져 있으면:

- 카메라 영상(노트북↔Pi, AP 자신과의 통신)은 **정상 작동**
- 모터 명령/센서 스트림(노트북↔ESP32, 클라이언트간 직접 통신)은
  **조용히 전부 막힘**

증상이 "화면은 나오는데 로봇이 안 움직이고 센서값도 안 들어온다"로
나타나서, IP 문제(위)와 헷갈리기 쉽다. 라즈베리파이 핫스팟 설정에서
client isolation이 꺼져 있는지 배포 전 확인할 것.

## 요약

| 구간 | 방식 | 배포 시 재확인 필요 |
|---|---|---|
| 노트북 ↔ 라즈베리파이 (카메라) | Pi가 서버, 노트북이 클라이언트 | `PI_IP` |
| 노트북 ↔ ESP32 (모터/센서) | 직접 P2P (Pi 안 거침) | `PC_IP`/`ESP32_IP`, `.ino`의 `SERVER_IP`, client isolation 여부 |
| 노트북/ESP32 ↔ 라즈베리파이 (네트워크) | Pi가 WiFi 핫스팟 호스트 | 핫스팟 서브넷 자체가 `192.168.137.x` 예전 가정과 다를 가능성 높음 |
