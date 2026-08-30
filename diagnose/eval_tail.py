"""0 kg 외란 튜닝 평가 — 복귀 하나만 보면 교환이 안 보인다.

`verify_worstcase` 의 복귀[s] 는 |y|<2cm 절대 밴드를 언제 통과하냐라서,
튜닝이 무엇을 사고 무엇을 팔았는지가 그 한 숫자에 안 담긴다. 이 스크립트는
같은 ts CSV 에서 네 축을 같이 뽑는다:

  경로복귀   펄스 끝 -> y 0 교차. **권한**이 정상인지 (크기와 무관해야 정상)
  오버슈트   0 교차 뒤 반대쪽 최대. 여기가 크면 2cm 밴드 밖에서 시작한다
  꼬리 시정수 오버슈트가 빠지는 속도. 08-29 기준 ~8 s
  호버 지터  펄스 전(t<2.9) 자세 RMS. **깎기의 대가가 나오는 자리**

사용: python eval_tail.py ts_0kg_U.csv [...]
"""
import csv, math, sys

def load(path):
    d = {k: [] for k in ('t','x','y','z','xref','roll','pitch','yaw')}
    with open(path) as f:
        for r in csv.DictReader(f):
            for k in d: d[k].append(float(r[k]))
    return d

def at(t, a, tx):
    i = min(range(len(t)), key=lambda k: abs(t[k]-tx)); return a[i]

def report(path):
    d = load(path); t, y = d['t'], d['y']; D = math.degrees
    ip = max(range(len(y)), key=lambda i: abs(y[i]))
    peak = y[ip]
    # 펄스 끝: 피크 직전 자세가 급변한 지점 대신, 관례상 T0+TM/2+DUR 를 쓰지 않고
    # 데이터에서 찾는다 (배율·질량이 바뀌면 시각이 달라지므로).
    tz = next((t[i] for i in range(ip, len(t)) if abs(y[i]) < 0.002), float('nan'))
    seg = [(t[i], y[i]) for i in range(len(t)) if t[i] > tz]
    ov = max(seg, key=lambda p: abs(p[1]))[1] if seg else float('nan')
    a, b = abs(at(t,y,9.0)), abs(at(t,y,15.5))
    tau = 6.5/math.log(a/b) if b > 0 and a > b else float('nan')
    pre = [i for i,v in enumerate(t) if 1.0 < v < 2.9]
    jit = math.sqrt(sum(D(d['roll'][i])**2 + D(d['pitch'][i])**2 for i in pre)/(2*len(pre))) if pre else float('nan')
    xov = max((abs(d['x'][i]-d['xref'][i]) for i,v in enumerate(t) if v > tz), default=float('nan'))
    print(f'{path.split("ts_")[-1]:22s} 밀림 {100*peak:7.2f} cm | 경로복귀 {tz:5.2f} s | '
          f'오버슈트 {100*ov:5.2f} cm | 꼬리 {tau:5.2f} s | 호버지터 {jit:.4f} deg | x잔차 {100*xov:5.2f} cm')

for p in sys.argv[1:]:
    try: report(p)
    except Exception as e: print(f'{p}: {e}')
