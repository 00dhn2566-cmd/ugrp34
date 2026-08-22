# overrides — 팀 코드를 우리 입맛대로 고친 버전

**원본은 한 줄도 안 고칩니다.** 여기 있는 건 원본 옆에 나란히 놓고 숫자를 재기 위한
우리 버전입니다. 숫자가 좋으면 근거를 들고 제안하면 되고, 아니면 여기만 버리면 됩니다.

```bash
python scripts/compare_recon.py --seeds 1 3 5 7 11
```

## 왜 만들었나

복원 오차를 뜯어보니 **복원단에 안전장치가 하나도 없었습니다.**

태민의 [`window_recon_node.py:46-49`](../../visual_imaging_taemin/window_recon_node.py#L46-L49):

```python
M = np.eye(3) - np.outer(d, d)
self.A += M          # 계수 없음 — 모든 관측이 동등하게 1표
self.b += M @ c
```

이건 $\min_p \sum_k \lVert (I - d_kd_k^\top)(p - c_k)\rVert^2$, 순수 비가중 L2입니다.
그 결과:

| 구멍 | 증상 |
|---|---|
| conf가 문턱 통과 뒤 버려짐 | conf 0.71과 0.99가 같은 표 |
| 중복 검출 = 중복 표 | 한 프레임에서 blue 창문 하나에 blue 박스 3개 → 3표 |
| 색 오분류 = 통째로 다른 바구니 | green 복원이 blue 위치로 끌려감 (857 mm) |
| 아웃라이어 제거 0 | L2는 breakdown point 0 — 나쁜 광선 하나가 제곱으로 당김 |

길남의 [`eval_recon3d._triangulate`](../../overall_gilnam/vision/eval_recon3d.py#L66)는
쌍별 삼각측량 + 성분별 median이라 아웃라이어에는 강건하지만, 역시 conf 가중은 없고
비용이 $O(N^2)$입니다.

## 무엇을 바꿨나

### `detections.py` — 검출 스트림 후처리

팀 코드에 §5 메시지를 **넘기기 전에** 우리가 거르는 층입니다. 그래서 태민·길남
양쪽 소비자에게 동시에 먹습니다. 팀 파일 관점에서는 그냥 입력이 깔끔해진 것뿐입니다.

- `top1_per_colour` — 프레임당 색별 conf 최고 1개만. 중복 표 제거
- `drop_tiny` — 몇 픽셀짜리 박스 제거 (시선 각도 오차가 커서 삼각측량에 독)
- `min_conf` — 태민 노드 문턱 전에 우리가 더 세게 거는 용도

> 주의: `top1_per_colour`는 **색 = 창문 고유 식별자**라는 씬 가정에 기댑니다.
> 같은 색 창문이 둘 이상이면 쓰면 안 됩니다.

### `recon_rays.py` — 태민 수치 경로

| # | 바꾼 것 | 이유 |
|---|---|---|
| 1 | conf 가중 `A += w·M` | conf를 문턱 뒤에도 씀 |
| 2 | **Huber IRLS** | 잔차 기반 재가중으로 섞여든 광선을 죽임 |
| 3 | 크기에 코너 4개 다 사용 | 원본은 `w=|TR−TL|`, `h=|BR−TR|` 로 한 변씩만 |
| 4 | `n_rejected` / `inlier_frac` 반환 | 뭐가 얼마나 버려졌는지 안 보이면 튜닝 불가 |

**일부러 안 바꾼 것**: 채택 게이트(코너 4개 + 시차각 ≥ 2°), `T_IC` 적용 방식,
반환 dict 키. 하류가 구분 없이 먹어야 갈아끼울 수 있습니다.

**포기한 것**: 원본의 $O(1)$ 증분 누적. IRLS는 재가중하려면 광선을 다 들고 있어야
해서 메모리가 $O(N)$입니다. 오프라인 리포트에선 무의미한 비용이지만, 그가 30Hz로
돌리는 노드에 그대로 넣을 물건은 아닙니다. 제안한다면 "리포트 주기에만 IRLS 한 번"
형태입니다.

## 측정 결과

seed 5, 72프레임, **동일한 관측**에 복원 방식만 바꿔 넣은 것입니다 (검출 랜덤성 배제).

```
              복원   center 중앙값  center p90  center 평균  size 중앙값
taemin        3/3        268 mm      420 mm      265 mm     147 mm
ours-same     3/3        269 mm      421 mm      266 mm     147 mm   ← 재구현 검증
ours-dedup    3/3        203 mm      408 mm      225 mm     147 mm
ours-conf     3/3        195 mm      367 mm      206 mm      78 mm
ours-full     3/3         63 mm      171 mm       91 mm      42 mm
```

창문별로는 `red 71→12`, `green 458→198`, `blue 268→63 mm`.

**`ours-same`이 `taemin`과 1mm 이내로 일치**하는 게 중요합니다 — 우리 재구현이
그의 알고리즘과 같은 답을 낸다는 뜻이고, 그래야 나머지 행의 차이가 순수하게
우리가 추가한 것에서 왔다고 말할 수 있습니다.

(완전 일치가 아닌 이유: 원본은 ROS pose 버퍼에서 20ms 내 최근접 pose를 찾고,
우리 재구현은 샘플의 pose를 직접 씁니다. 보간 경로 차이입니다.)

## 원본과 동일하게 돌리려면

```python
recon_rays.reconstruct(samples, weight="none", robust="none", size="taemin")
# 또는
recon_rays.reconstruct_like_taemin(samples)
```
