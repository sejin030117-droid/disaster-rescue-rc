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

> ★**2026-08-27 정정**: 아래 "모터 명령 / ToF·온도·가스 센서 스트림"
> 항목이 모터 명령도 TCP(P2P)로 간다고 적었는데, 실제 이동제어 `.ino`
> 는 **Bluetooth Classic(SPP) + 단일 문자**를 쓴다(TCP/JSON 아님). 이
> 블루투스 수동조종은 **설계상 유지하기로 확정**했다(나중에 수동조종
> 시연 용도). 자세한 내용·남은 작업은 §"이동 제어 / 경로 실행" 절의
> "★모터 명령 경로 — 확인 완료" 항목 참고. 아래 표의 8890 행은 현재
> **미사용 중(펌웨어가 그 프로토콜을 모름)** - 실제로 쓰려면 위 절
> "남은 작업 ③" 대로 정리가 먼저 필요하다.

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

## ★정정 노트 (2026-08-20) — 다른 AI가 준 통신구조 요약 중 틀린 부분

외부(다른 AI)에서 받은 통신구조 요약에 아래 두 가지 주장이 있었는데,
**둘 다 이 저장소의 코드 및 위 섹션들과 대조한 결과 부정확한 것으로
확인됨** — 앞으로 비슷한 요약을 다시 받더라도 이 두 항목은 무시할 것.

1. **"ESP32가 서버(라즈베리파이)를 다시 찾아간다"는 주장 — 틀림.**
   위 "★IP 설정" 섹션의 실제 `.ino` 스니펫을 보면
   `SERVER_IP = "192.168.137.1"`(노트북)이 기본 타겟이고, Pi(`.52`)는
   주석에 "Pi로 테스트하려면"이라고만 적힌 **대안 테스트 타겟**이다.
   즉 ESP32의 실제 통신 상대는 라즈베리파이가 아니라 **노트북**이다
   (`robot_config.STREAM_PORT=9999`도 `"ESP32 -> PC (PC가 서버)"`로
   명시). Pi는 AP(무선 신호 중계)일 뿐 TCP 엔드포인트가 아니다. 재연결
   로직도 "서버를 동적으로 탐색"하는 게 아니라 하드코딩된 고정 IP에
   3초 간격으로 단순 재시도하는 방식이며, 이 스니펫에는 진짜 하드웨어
   워치독(`esp_task_wdt` 등)도 보이지 않는다 — 프로덕션 `.ino`에 별도로
   있을 수는 있으나 이 테스트 스케치만으로는 확인 불가.

2. **"라즈베리파이가 GPIO/I2C로 하드웨어(모터/센서)를 직접 제어한다"는
   주장 — 틀림 (과거 구상 단계 아이디어였을 뿐 현재 설계 아님).**
   모터·ToF·온도/가스 센서 제어는 지금도 **ESP32가 전담**한다. 이
   저장소의 라즈베리파이 쪽 코드는 카메라 스트림 송출(`rpicam-vid` →
   TCP 8888)뿐이고, 로컬 GPIO/I2C 하드웨어 제어 스크립트는 없다 — 없는
   게 정상이다.

3. **IP 대역 리스크는 여전히 미해결 상태다** (위 "★IP 설정" 섹션과
   동일) — `ip addr show wlan0` 실측이 아직 안 됐으므로 "해결됨"으로
   바꾸지 말 것. `192.168.137.x`가 실제 Pi 핫스팟 서브넷과 다를 가능성
   (`10.42.0.x` 등)이 여전히 가장 유력한 미확인 리스크로 남아있다.

---

# ESP32 핀 배치 (2026-08-26 확정)

> 실물 배선 기준. 이 표가 단일 출처이며, `.ino` 의 핀 상수는 여기서
> 가져온다. 배선을 바꾸면 **이 표를 먼저 고치고** `.ino` 를 맞출 것.

| 핀 | 연결 | 이 파일에서 쓰는가 |
|---|---|---|
| 4  | MPU6050 INT | ✗ (`MPU6050_light` 는 폴링 방식이라 INT 안 씀) |
| 13 | DHT22 (온도) | ○ |
| 14 | Servo (ToF·카메라 마운트) | ○ |
| 16 | R_BTS R_EN | △ 부팅 시 LOW 로만 내림 |
| 17 | R_BTS L_EN | △ 〃 |
| 18 | 오른쪽 Encoder A | ✗ (이동 제어와 함께 별도 파일) |
| 19 | R_BTS LPWM | △ 부팅 시 LOW |
| 21 | **I2C SDA — LCD + MPU6050 + ToF 공유** | ○ |
| 22 | **I2C SCL — LCD + MPU6050 + ToF 공유** | ○ |
| 23 | 오른쪽 Encoder B | ✗ |
| 25 | L_BTS R_EN | △ 부팅 시 LOW |
| 26 | L_BTS L_EN | △ 〃 |
| 27 | L_BTS RPWM | △ 〃 |
| 32 | R_BTS RPWM | △ 〃 |
| 33 | L_BTS LPWM | △ 〃 |
| 34 | 왼쪽 Encoder A | ✗ (34/35 는 **입력 전용** 핀) |
| 35 | 왼쪽 Encoder B | ✗ 〃 |

**△ 표시의 의미**: `rescue_sensor_node.ino` 는 이동을 다루지 않지만,
BTS7960 의 EN 핀이 뜬 상태로 방치되면 예기치 않게 모터가 돌 수 있다.
그래서 부팅 시 8개 모터핀을 전부 `OUTPUT` + `LOW` 로 내려 "확실히 정지"
상태만 만들어 둔다. 구동 로직은 넣지 않는다.

## ★가스 센서 핀이 없다 — 미해결

위 핀맵에 **가스 센서 자리가 없다.** 그런데 `robot_config.GAS_PPM=40`
위험판정과 스트림 프로토콜의 `gas` 필드는 이미 존재한다. 지금은
`.ino` 가 `gas` 필드를 아예 안 보내고, 파이썬(`route_message`)이 그걸
`None` 으로 처리해서 조용히 넘어간다 — **즉 안 깨지지만 가스 위험이
영영 안 뜬다.** 핀이 정해지면 `.ino` 에 읽기 코드를 추가하고 스트림에
필드를 넣을 것.

## ★온도가 파이썬으로 안 넘어간다 — 미해결

`.ino` 는 DHT22 값을 `temp` 필드로 보내는데,
`tof_commander.route_message()` 가 `dist/servo/st/gas/yaw` 만 꺼내고
**`temp` 는 꺼내지 않는다.** `FIRE_TEMP_C=50` 화재판정에 필요한 값이라
파이썬 쪽에 한 줄 추가가 필요하다. (§2 임계값 단일출처화와 같은 계열
— 상수는 통일했는데 데이터 경로가 안 이어져 있는 상태.)

## 펌웨어 파일 / 빌드

- 파일: `rescue_sensor_node/rescue_sensor_node.ino`
  (Arduino 규칙상 **폴더명 == 파일명** 이어야 IDE 가 인식한다)
- 담당: 통신 2채널 + 서보 스윕 + 센서 스트림 + `aim`/`scan` 명령 처리.
  **이동 제어는 없다.**
- 스윕 파라미터(`SWEEP_STEP_DEG` 등)는 파일 최상단
  **[사용자 조정 구역]** 에 전역변수로 모아뒀다.
- 라이브러리: `ESP32Servo`, `Adafruit_VL53L0X`, `MPU6050_light`,
  `DHT sensor library` (전부 설치 완료, 2026-08-26)
- 코어: `esp32:esp32` **3.3.11**. 이 버전부터 `WiFiServer::available()`
  이 deprecated 라 `accept()` 를 쓴다 — 2.x 로 내리면 되돌려야 한다.

빌드 검증(2026-08-26 통과, 플래시 74% / RAM 15%):

```bash
"$LOCALAPPDATA/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe" compile --warnings all --fqbn esp32:esp32:esp32 rescue_sensor_node
```

VS Code 에서는 `Ctrl+Shift+B` 로 같은 컴파일이 돈다
(`.vscode/tasks.json`, `problemMatcher` 가 에러를 줄번호까지 잡아줌).
Microsoft 의 VS Code Arduino 확장은 2024-10-01 마켓플레이스에서
삭제됐으므로 설치하려 하지 말 것 — `arduino-cli` 직접 호출이 정답이다.

## 실물 확인이 아직 안 된 것 (컴파일 통과 ≠ 동작)

1. 서보 펄스폭 `500~2500us` 가 실제 서보 모델에 맞는지
2. I2C 3개(LCD/MPU6050/ToF) 주소 충돌 여부 — 21/22 를 공유한다
3. 각도 부호 — 오른쪽이 `+` 인지 실물로 돌려봐야 확정
   (`SERVO_SIGN`, `robot_config.ESP32_SERVO_*` 사본과 맞출 것)
4. 위 "★IP 설정" 의 핫스팟 대역 문제 — 첫 부팅 시리얼에 찍히는
   `[WiFi] 연결됨. 내 IP = ...` 로 바로 판가름난다

---

# 이동 제어 / 경로 실행 (2026-08-26)

> 격자 이동을 "한 칸씩" 대신 "필요한 거리만큼 한 번에" 로 바꾸는 논의에서
> 파생된 조사 결과. 시뮬(`path_planner_sim2.py`)과 실기(`motor_commander.py`
> + `.ino`) 양쪽의 이동 표현이 서로 어긋나 있는 지점들을 정리한다.

## 현재 상태 한눈에

| 항목 | 상태 | 근거 |
|---|---|---|
| 로봇 위치 표현 | 정수 셀 전용 | `robot_x, robot_y = nxt_x, nxt_y` (`path_planner_sim2.py:1933`) |
| 소수 좌표 렌더링 | ✅ 이미 됨 | `MapCanvas._pt()` → `QPointF` (`map_canvas.py:202`) |
| 임의 각도 몸통 회전 | ✅ 이미 실행 중 | `turn_body_to_reach()` (`:674`) — 조준용으로 이미 실수각을 씀 |
| 임의 길이 직선 안전검사 | ✅ 이미 있음 | `segment_safe()` (`path_planner.py:188`) — float 끝점·길이무제한 |
| 플래너의 임의 헤딩 수용 | ✅ 이미 상정 | `bfs_from()` 의 `start_diffs` (`:1307`) |
| mm/deg 단위 명령 래퍼 | ✅ 이미 있음 | `motor_commander.move_forward(mm)` / `turn_left(deg)` |
| **경로 스무딩(웨이포인트 병합)** | ❌ **없음** | 이게 유일하게 빠진 조각 |

즉 "한 번에 멀리 가기" 에 필요한 부품은 대부분 이미 있고, **플래너가
A* 결과를 한 칸씩 그대로 실행하는 것만 남아 있다.**

## 🐛 `duration_ms: 200` 고정 — 명백한 누락

`path_planner_sim2.py` 의 `send_command()` 호출부는 3곳뿐인데, 회전
2곳은 각도 비례인 반면 직진만 상수다.

```python
:694   "duration_ms": int(abs(need) * 10)   # 조준 회전 — 각도 비례 O
:1925  "duration_ms": int(abs(diff) * 10)   # 이동 회전 — 각도 비례 O
:1931  "duration_ms": 200                   # 직진 — 고정 X
```

그런데 `MOVES_16` 의 한 스텝은 **50 / 70.7 / 111.8mm 세 종류**다
(직진 / 대각 / 나이트). 셋 다 같은 시간 전진하므로 실기에서는 즉시
어긋난다. 설계 의도가 아니라 빠뜨린 것으로 판단된다.

**고칠 방향**: dict 를 직접 조립하지 말고 `motor_commander` 의 의미
단위 래퍼(`move_forward(mm)` / `turn_left(deg)`)를 타게 할 것. 그러면
보정식이 `motor_commander.py` 한 곳에만 남는다(§2 임계값 단일출처화와
같은 계열).

## ★시뮬과 실기의 속도 상수가 다르다 — 미해결

같은 로봇의 속도가 두 파일에서 다르게 가정돼 있다.

| | 시뮬 (`path_planner_sim2.py`) | 실기 (`motor_commander.py`) | 차이 |
|---|---|---|---|
| 직진 | `ROBOT_SPEED_MPS = 0.15` → 150 mm/s | `distance_mm * 4` → 250 mm/s | 1.67배 |
| 회전 | `ROBOT_TURN_DPS = 75.0` → 75 °/s | `angle_deg * 10` → 100 °/s | 1.33배 |

시뮬이 계산한 미션 소요시간(`sim_time_s`)이 실기와 안 맞는다는 뜻이다.
**어느 쪽도 실측값이 아니므로 지금은 둘 다 추정치다.**

펌웨어(`rescue_sensor_node.ino`)를 대조하니 센싱 쪽도 같은 문제가 있다.

| | 시뮬 | 펌웨어 | 차이 |
|---|---|---|---|
| 스윕 각속도 | `SERVO_SWEEP_DPS = 100` | 5도/`SWEEP_SETTLE_MS`(60ms) → **≤83.3 °/s** (서보 이동시간 미포함이라 실제는 더 느림) | 시뮬이 빠름 |
| ToF 측정주기 | `TOF_PERIOD_S = 0.020` (50Hz) | 스텝당 1회 → **~16.7Hz** | **시뮬이 3배** |
| 서보 가동범위 | `SERVO_MAX_OFF = 65` | `SWEEP_MIN/MAX_OFF = ±80` | 실기가 넓음 |

★서보 범위는 **실측이 아니라 배선만 고치면 되는 건**이다 —
`robot_config.py` 가 이미 `ESP32_SERVO_MIN/MAX_OFF = ±80` 으로 정해
두었는데 시뮬만 65 를 쓴다. 시뮬 주석은 `(robot_config 서보범위)` 라고
적어놓고 값이 안 맞는 상태다.

**재야 할 목록은 `robot_config.py` 의 "이동 / 스윕 속도" 절에 모아 두었다**
(2026-08-26 추가). ★소비처 배선은 실측 전까지 건드리지 말 것 — 시뮬 값을
바꾸면 그동안 쌓은 A/B 측정(경로 스무딩 등)이 무의미해진다.
실측 → 값 확정 → 배선 순서를 지킬 것.

또한 `motor_commander` 의 `distance_mm * 4` 처럼 **원점을 지나는 비례식은
실제로 잘 안 맞는다** — 정지마찰 극복 + 가속 구간 때문에 실제 관계는
`duration_ms = distance_mm * K + C` (절편 C = 가속 손실) 에 가깝다.
짧은 이동일수록 이 오차 비중이 커진다(= 한 번에 멀리 가는 쪽이 유리한
또 하나의 이유). 실측 시 100/200/400/800ms 를 각 5회씩 재서 직선
피팅으로 K 와 C 를 함께 뽑을 것.

## ★엔코더 — 하드웨어 미완성 (2026-08-26 사용자 확인)

핀 배치표(위 "ESP32 핀 배치")에 엔코더 4핀이 잡혀 있어서 "배선은 됐고
코드만 없다" 로 읽히기 쉬운데, **실제로는 하드웨어 자체가 아직 미완성
상태다.** 구현 방법도 미정.

- 핀: R = 18(A)/23(B), L = 34(A)/35(B) — 좌우 A/B 2채널 = 직교
  방식이라 완성되면 방향 판별까지 가능
- `rescue_sensor_node.ino` 는 **센서 전용 노드**다. 엔코더도 모터도
  구동하지 않으며(`PIN_ENC_*` 는 선언만 되고 파일 어디서도 안 쓰임),
  주석이 "이동 제어와 함께 별도 파일에서 다룬다" 고 명시한다. 그
  "별도 파일" 은 이 저장소에 없다.
- 스트림(`sendStream()`)에 엔코더/틱 필드가 없다. `yaw`(자이로)는 있다.

**결론적으로 각도는 자이로로 실측 가능하지만 거리는 실측 수단이 없다** —
비대칭 상태다. 이 때문에 지금은 "거리를 시간으로 번역해서" 명령한다
(위 속도 상수 항목 참고). 엔코더가 완성되면 이 번역 자체가 사라지고
`{"cmd":"forward","distance_mm":130}` 형태로 바뀔 수 있다.

★이 항목은 **인지만 하고 지금은 건드리지 않는다** — 아래 "다음 작업" 의
스무딩은 하드웨어와 무관하게 진행 가능하다.

**착수할 때 볼 것: `odometry_plan.md`** — 실측 절차, 코드 변경 지점,
차동구동 공식, 엔코더로도 안 풀리는 것까지 정리해 두었다.

## ★모터 명령 경로 — 확인 완료 (2026-08-27), 정정 필요

이전 기록(아래 참고용으로 남김)은 "TCP 8890 으로 보내는데 안 받는다"고
추정했으나, **실제 이동제어 `.ino`(사용자 제공, RC카 수동조종용 검증
스케치)를 보니 전제 자체가 달랐다.**

**① 통신 방식이 TCP/JSON 이 아니라 Bluetooth Classic(SPP) + 단일 문자다**

```cpp
BluetoothSerial SerialBT;
const char* BLUETOOTH_NAME = "ESP32_RC";
...
case 'F': /* 전진 시작 */   case 'B': /* 후진 */
case 'L': /* 우회전 90도 */ case 'R': /* 좌회전 90도 */  // ★매핑 반전 주석 있음, .ino 주석 신뢰
case 'X': /* 정지 */
case 'I'/'K': /* 왼쪽 base PWM ±10 */   case 'O'/'P': /* 오른쪽 base PWM ±10 */
```

포트 8890, JSON, `duration_ms` 같은 개념이 아예 없다. `motor_commander.py`
(TCP+JSON)와 이 펌웨어(BT+문자)는 지금 그대로 두면 서로 말이 안 통한다 -
"무시된다"가 아니라 "채널 자체가 다르다" 가 정확한 진단이다.

**② ESP32 는 한 대이고, 두 펌웨어가 핀을 전부 공유해서 배타적이다**

이동제어 `.ino` 의 모터/MPU/I2C 핀이 `rescue_sensor_node.ino` 와 정확히
같다(L: 25/26/27/33, R: 16/17/32/19, MPU INT: 4, I2C: 21/22). 즉
"한 대 + 펌웨어 두 개" 가 맞았고, **지금은 둘 중 하나만 올릴 수 있다** -
합치거나 역할을 나눠야 한다.

**③ 차동 PWM 명령은 이미 있다 - "차동 속도 명령이 없다"던 이전 판단은 틀렸다**

```cpp
leftAppliedPWM  = constrain(leftBaseSpeed  - directional, 0, 255);
rightAppliedPWM = constrain(rightBaseSpeed + directional, 0, 255);
```

자이로(MPU6050 DMP) 기반 Yaw PID 가 직진 중 좌우 바퀴에 이미 다른 PWM 을
주고 있다(호 주행 궤적 오차 이야기의 전제가 이걸로 약해진다 - 이 로봇은
직진 중에도 자세를 스스로 잡는다). `targetYaw` 를 시간에 따라 서서히
바꾸기만 하면 기존 PID 로 호 주행이 그대로 나온다 - 새 메커니즘이
필요 없다.

**④ 회전은 ±90도 고정, 임의 각도 불가**

`LEFT_TURN_ANGLE`/`RIGHT_TURN_ANGLE` 이 상수다. 2단계 제어(거친 접근 →
±1도 미세 펄스, 정착대기 300ms, 8초 타임아웃)는 잘 만들어져 있어서,
스무딩이 요구하는 임의 각도(18.43도 등)를 쓰려면 이 상수를 함수 인자로
바꾸는 정도로 충분해 보인다 - 제어 루프 자체는 안 바꿔도 될 가능성이 높다.

**⑤ 거리 개념이 아예 없다 - 실수 좌표 이동의 진짜 병목**

`'F'` 는 `'X'` 를 받을 때까지 무한 전진하는 상태(`STATE_FORWARD`)로
들어갈 뿐, 시간 제한도 엔코더도 없다. "130mm 가라" 를 표현할 방법이
현재 이 펌웨어엔 없다 - `motor_commander.move_forward(mm)` 같은 함수를
받아줄 종료 조건이 없다는 뜻. 엔코더 미완성과 별개로, **거리 기반 종료
조건 자체를 먼저 추가해야** `duration_ms` 근사든 엔코더 폐루프든 걸 자리가
생긴다.

**⑥ ★BT Classic + WiFi 동시 사용 - 실제 컴파일로 검증 완료**

기본 파티션(1.3MB)으로는 `rescue_sensor_node.ino` 의 전체 라이브러리
세트(WiFi/VL53L0X/MPU6050_light/DHT/ESP32Servo) + `BluetoothSerial` 을
한 스케치에 넣으면 플래시 초과로 빌드가 실패한다
(`text section exceeds available space`). **`PartitionScheme=huge_app`
(3MB) 로 바꾸면 빌드된다** - 실측: 플래시 1,666,675B(52%), RAM
65,744B(20%), 48% 여유.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app rescue_sensor_node
```

★대가: OTA 업데이트 불가(USB 플래시만). Arduino IDE 에서는 도구 →
Partition Scheme → `Huge APP (3MB No OTA/1MB SPIFFS)`.

★런타임 비용은 컴파일로는 안 잡힌다 - BT/WiFi 가 같은 2.4GHz 안테나를
시분할하므로 ToF 스트림(16.7Hz) 지연 등은 실측 필요. RAM 20% 는
컴파일타임 수치이고 BT/WiFi 스택이 런타임에 힙을 추가로 잡는다(BT
Classic 만 대략 30~50KB) - 여유(261KB)는 있어 보이지만 확정은 아니다.

**★설계 결정 (2026-08-27, 사용자)**: 블루투스 수동조종을 **없애지 않고
가져가는 방향으로 확정**한다 - 나중에 "수동으로도 조종 가능함" 을
시연하는 용도로 쓸 것. 즉 위 ⑥의 BT+WiFi 공존이 임시 우회가 아니라
최종 설계에 필요하다. `huge_app` 파티션 전제로 진행할 것.

**남은 작업(우선순위순)**:
1. 이동제어 `.ino` 에 거리 기반 종료 조건 추가(⑤) - 이게 있어야 아래
   전부가 의미를 가진다
2. 두 펌웨어 합치기(②) - 센서(WiFi/TCP) + 이동(BT/문자) 를 한 스케치로,
   `huge_app` 파티션 전제
3. PC 쪽 프로토콜 정리 - `motor_commander.py` 가 TCP 대신(또는 병행)
   BT SPP 로 이 문자 프로토콜을 말하게 할지, 아니면 이동제어 쪽에 JSON
   커맨드 해석을 얹어 기존 `motor_commander.py` 를 그대로 살릴지 결정
   필요 - 어느 쪽이든 §"통신 구조" 절의 포트맵/토폴로지 갱신 필요
4. 회전 각도 파라미터화(④), 호 주행 시험(③)
5. 실측: BT+WiFi 동시 구동 시 ToF 스트림 지연/드롭률

<details>
<summary>이전 기록 (2026-08-26, TCP 단일 채널로 오판했던 버전 - 참고용)</summary>

`motor_commander.py` 는 `CMD_PORT`(8890)로 `forward`/`turn_left`/
`turn_right` 를 보낸다. 그런데 같은 8890 을 여는 `rescue_sensor_node.ino`
의 `handleCommand()` 가 아는 명령은 **`sweep`/`hold`/`aim`/`scan`/`ping`**
뿐이다 - 모터 명령은 조용히 무시된다.

가능성 두 가지이며 아직 확인 안 됨:
- (a) ESP32 **한 대 + 펌웨어 두 개** → 합쳐야 함 (주석의 "별도 파일" 이
  이쪽을 시사)
- (b) ESP32 **두 대** → 그런데 `robot_config.py` 에 `ESP32_IP` 가 하나뿐이라
  IP 가 부족하다

이 진단은 절반만 맞았다 - (a)는 맞았지만, 안 통하는 이유가 "명령 종류가
안 맞아서" 가 아니라 "프로토콜 자체가 다른 채널(BT vs TCP)" 이었다.
</details>

## 참고 — `socket_receiver` 가 stream 페이로드를 버린다

`socket_receiver.py:97` 의 stream 분기는 카운터만 올리고 필드를 하나도
꺼내지 않는다.

```python
elif t == "stream":
    self.stat["stream"] += 1      # 페이로드 폐기
```

나중에 엔코더 필드를 스트림에 실어도 파이썬이 버린다. 위 §"온도가
파이썬으로 안 넘어간다"(`tof_commander.route_message`)와 별개 파일이지만
같은 종류의 누락이다.

## 소수 좌표로 바꿀 때 깨지는 곳 — 딱 2곳

전수 조사 결과, 대부분은 이미 float 안전하다(`has_line_of_sight`,
`find_nearest_unknown`, `find_confirm_point`, `real_scan`, `record_hit`,
`bump_cells` 전부 내부에서 `int(round())` 하거나 순수 산술만 쓴다 —
`sweep_while_moving` 이 이동 중 소수 포즈로 센싱하느라 이미 그렇게
만들어져 있다). 깨지는 건 다음 2곳뿐이다.

| 위치 | 증상 | 고칠 방법 |
|---|---|---|
| `bfs_from()` 호출부 (`:1740`, `:1895`) | `TypeError` — `si = sy*GRID_SIZE+sx` 가 배열 인덱스 | 호출부에서 `int()` |
| `(robot_x, robot_y) == home` (`:1885`) | ★**조용히** 영원히 False → 복귀 완료 판정 불가 | `math.hypot(...) < 0.5` 거리 판정으로 |

두 번째가 위험하다 — 에러 없이 로봇이 집 앞에서 계속 맴돈다.

★단, 좌표 실수화는 **엔코더가 생긴 뒤에야 의미가 있다**(지금 구조에선
거의 순수 표시용). 이유와 착수 절차는 `odometry_plan.md` §2 참고.

## 다음 작업 — 경로 스무딩부터

결론: **스무딩은 소수 좌표 전환 없이도 가능하다.** 웨이포인트를 병합해도
끝점은 여전히 정수 셀이고(A* 셀), 달라지는 건 스텝 길이와 헤딩뿐인데
헤딩은 `turn_body_to_reach` 가 이미 실수각을 처리한다. 즉 **기존 불변식을
하나도 안 건드린다** — 위 "깨지는 2곳" 도 스무딩 단계에선 안 건드려도 된다.

권장 순서:

1. **경로 스무딩** — `segment_safe()` 로 검사하며 웨이포인트 병합.
   하드웨어 무관, 기존 불변식 무손상. 이득의 대부분이 여기서 나온다.
2. **소수 포즈 + 렌더링 보간** — 1 을 하면 긴 스텝이 화면에서 순간이동해
   오히려 끊겨 보인다. 그때 중간 포즈 보간을 넣는다(`sweep_while_moving`
   이 이미 계산 중인 값). 이때 위 "깨지는 2곳" 수정이 필요해진다.
3. **명령 계층 정리** — `move_forward(mm)`/`turn_left(deg)` 로 통일하고
   속도 상수를 `robot_config.py` 로 단일출처화. **값 실측은 하드웨어가
   준비된 뒤 그 한 곳만** 고치면 된다.
4. (하드웨어) 엔코더 → `MM_PER_TICK` 실측 → 폐루프.

★**시뮬 낚임 방지**: 시뮬은 포즈가 곧 진실이라 스무딩이 실제보다 좋아
보인다. 병합 최대 길이 상한을 `robot_config.py` 의 튜닝 상수로 빼고
**처음엔 보수적으로** 잡을 것. 시뮬 수치는 "이득의 방향" 으로만 읽고
"실물 예측치" 로 읽지 말 것.
