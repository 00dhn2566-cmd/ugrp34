function qc_mhe_sfun(block)
%QC_MHE_SFUN  지연 보상 MHE — Level-2 MATLAB S-Function. 메모리 수술로 삽입, save 금지.
%
%   입력 1: 지연된 측정 위치 [x y z] (Meas Delay Pos 출력)
%   입력 2: 측정 자세 [roll pitch yaw]  (Goto/From 으로 건너온 것)
%   출력 1: 보정된 위치 [x y z]  -> Position Control/Subtract2 입력 2
%
%   ★ direct feedthrough 를 **끈다**. 보상기가 제어 명령에 영향을 주고 그 명령이
%     다시 위치를 만드는 구조라, 켜 두면 대수 루프로 컴파일이 죽는다. 끄면 출력이
%     직전 스텝 상태에서 나오므로 한 스텝 지연이 생기는데, 1 kHz 에서 1 ms 는
%     보상하려는 지연(20~100 ms)의 1~5% 라 감수할 만하다.
%
%   ★ 무거운 해는 N 스텝마다만 돈다. 나머지는 마지막 바이어스로 전파만 한다 —
%     곱셈 몇 번이라 사실상 공짜다. 사이 구간을 ZOH 로 붙잡으면 N*dt 의 **새 지연**이
%     생겨 자기모순이므로 반드시 외삽한다.
%
%   base 워크스페이스 변수 (qc_mhe_defaults 가 채운다):
%     mhe_dt, mhe_horizon_s, mhe_meas_dt, mhe_meas_std, mhe_bias_rw,
%     mhe_prior_p_std, mhe_prior_v_std, mhe_tol_m, mhe_age_trust,
%     mhe_dropout_s, mhe_innov_max, mhe_v_max, mhe_dev_margin, mhe_g

setup(block);
end

function setup(block)
block.NumInputPorts  = 2;
block.NumOutputPorts = 1;
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
for k = 1:2
    block.InputPort(k).Dimensions        = 3;
    block.InputPort(k).DatatypeID        = 0;
    block.InputPort(k).Complexity        = 'Real';
    block.InputPort(k).DirectFeedthrough = false;   % ★ 대수 루프 방지
end
block.OutputPort(1).Dimensions = 3;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';

dt = evalin('base', 'mhe_dt');
block.SampleTimes = [dt 0];
block.NumDialogPrms = 0;
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Update',               @Update);
end

function DoPostPropSetup(block)
% 창 버퍼를 DWork 에 잡는다. persistent 를 쓰면 안 된다 — 한 프로세스에서 여러
% 시뮬을 연달아 돌릴 때 앞 실행의 상태가 새 실행으로 새어 들어온다 (이 저장소는
% 배치로 수십 번 돌리므로 실제로 문제가 된다).
nb = mhe_nbuf();
block.NumDworks = 6;
names = {'acc', 'meas', 'mvalid', 'state', 'aux', 'out'};
sizes = [3*nb, 3*nb, 3*nb, 9, 12, 3];
for k = 1:6
    block.Dwork(k).Name            = names{k};
    block.Dwork(k).Dimensions      = sizes(k);
    block.Dwork(k).DatatypeID      = 0;
    block.Dwork(k).Complexity      = 'Real';
    block.Dwork(k).UsedAsDiscState = true;
end
end

function n = mhe_nbuf()
% 창 길이를 스텝 수로. 창은 **지연이 아니라 관측 가능성**이 정한다 —
% 바이어스 열이 0.5*D^2 이라 창이 짧으면 바이어스가 측정 잡음에 묻힌다
% (H >~ sqrt(2*sigma/b), 08-29 실측: H=0.06 에서 추정 0.000, H=0.20 에서 0.954).
dt = evalin('base', 'mhe_dt');
H  = evalin('base', 'mhe_horizon_s');
n  = max(8, ceil(H / max(dt, 1e-9)) + 1);
end

function InitConditions(block)
nb = mhe_nbuf();
block.Dwork(1).Data = zeros(3*nb,1);      % acc
block.Dwork(2).Data = zeros(3*nb,1);      % meas
block.Dwork(3).Data = zeros(3*nb,1);      % mvalid
block.Dwork(4).Data = zeros(9,1);         % state: [p0 v0 b] x 3축
% aux: 1 head, 2 nfill, 3 since_solve, 4 N, 5 t_since_meas,
%      6..8 last_meas xyz, 9 last_meas_vt(상대), 10 have_meas, 11 solves, 12 fault
aux = zeros(12,1); aux(4) = 1;
block.Dwork(5).Data = aux;
block.Dwork(6).Data = zeros(3,1);         % 출력 (direct feedthrough 꺼짐)
end

function Outputs(block)
% ★ 여기서 입력을 읽으면 안 된다 (direct feedthrough = false). 직전 Update 가
%   계산해 둔 값을 그대로 낸다.
block.OutputPort(1).Data = block.Dwork(6).Data;
end

function Update(block)
dt   = evalin('base', 'mhe_dt');
nb   = mhe_nbuf();
cfg  = mhe_cfg();
mpos = block.InputPort(1).Data(:);        % 지연된 측정 위치
att  = block.InputPort(2).Data(:);        % [roll pitch yaw]

acc = block.Dwork(1).Data; meas = block.Dwork(2).Data;
mv  = block.Dwork(3).Data; st   = block.Dwork(4).Data;
aux = block.Dwork(5).Data;

% ── 자세 -> 월드 수평 가속도 (소각). 부호는 08-29 실측으로 확정:
%    a_x = +g*pitch (상관 +0.82), a_y = -g*roll (상관 -0.82). 교차항 무시 가능.
%    z 축은 자세로 안 만든다 — 고도 루프가 따로 있고 추력이 미지수라, 여기서는
%    바이어스가 z 의 모델 오차를 통째로 흡수하게 둔다 (a_z = 0).
cy = cos(att(3)); sy = sin(att(3));
g  = cfg.g;
a  = [ g*(att(2)*cy + att(1)*sy); ...
       g*(att(2)*sy - att(1)*cy); ...
       0 ];

% ── 링 버퍼 전진
% ★ 창이 꽉 차 있으면 t0 가 한 스텝 밀린다 = **이동 지평의 이동**. 그때 사전(prior)도
%   새 t0 로 전파해야 한다 — 파이썬 판은 predict_at/velocity_at 으로 그렇게 한다.
%   안 하면 한 스텝 낡은 상태를 사전으로 쓰게 되고, 사전 가중이 약해 치명적이진
%   않아도 명백한 이식 누락이다 (08-29 발견).
head = mod(aux(1), nb) + 1;
aux(1) = head;
if aux(2) >= nb
    % t0 가 dt 만큼 앞으로 간다: p0 <- p0 + v0*dt + 0.5*b*dt^2 + (그 구간 가속 적분)
    oldest = mod(head - nb, nb) + 1;          % 곧 버려질 슬롯 = 옛 t0
    for k = 1:3
        p0 = st((k-1)*3+1); v0 = st((k-1)*3+2); b = st((k-1)*3+3);
        a0 = acc((k-1)*nb + oldest);
        st((k-1)*3+1) = p0 + v0*dt + 0.5*(b + a0)*dt*dt;
        st((k-1)*3+2) = v0 + (b + a0)*dt;
    end
end
aux(2) = min(aux(2) + 1, nb);
for k = 1:3
    acc((k-1)*nb + head) = a(k);
    mv((k-1)*nb + head)  = 0;
end

% ── 측정 도착 판정. 값이 변했으면 새 표본이 온 것으로 본다 (지연 블록 출력은
%    측정 사이에 상수라 이 판정이 성립한다).
aux(5) = aux(5) + dt;
newmeas = false;
if aux(10) == 0
    newmeas = true;
elseif any(abs(mpos - aux(6:8)) > 1e-12)
    newmeas = true;
end

if newmeas
    % age_trust: 보고된 나이보다 **덜** 되감는다. 과소보상의 손해는 성능이지만
    % 과대보상은 추정을 실제보다 앞에 놓아 양의 되먹임 = 불안정이다.
    age = min(cfg.meas_age, cfg.horizon) * min(max(cfg.age_trust,0),1);
    back = min(round(age/dt), aux(2)-1);
    slot = mod(head - 1 - back, nb) + 1;
    for k = 1:3
        meas((k-1)*nb + slot) = mpos(k);
        mv((k-1)*nb + slot)   = 1;
    end
    aux(6:8) = mpos; aux(5) = 0; aux(10) = 1; aux(9) = back*dt;
end

% ── 무거운 해: N 스텝마다
aux(3) = aux(3) + 1;
if aux(3) >= aux(4)
    aux(3) = 0;
    n  = aux(2);
    idx = mod(head - n + (0:n-1), nb) + 1;   % 오래된 것부터
    trel = (0:n-1)' * dt;
    w = [1/cfg.prior_p^2; 1/cfg.prior_v^2; 1/cfg.bias_rw^2; 1/cfg.meas_std^2];
    dbmin = inf;
    for k = 1:3
        o = (k-1)*nb;
        [p0,v0,b,dbs,ok] = qc_mhe_solve(trel, acc(o+idx), meas(o+idx), ...
                                        mv(o+idx), st((k-1)*3+(1:3)), w);
        if ok
            st((k-1)*3+(1:3)) = [p0; v0; b];
            dbmin = min(dbmin, dbs);
        end
    end
    % ★ N 은 **정확도 예산**에서 나온다 (08-29 정정). 계산 예산으로 잡으면
    %   지연이 클수록 N 이 커져 — 제일 안 중요한 5 ms 에 계산을 제일 많이 쓰고
    %   생사가 갈리는 100 ms 에 제일 아끼는 — 결과와 반대 방향이 된다.
    %   0.5*db*(N*dt)^2 <= tol  ->  N <= sqrt(2*tol/db)/dt
    if isfinite(dbmin) && dbmin > 0
        aux(4) = max(1, min(cfg.n_max, floor(sqrt(2*cfg.tol/dbmin)/dt)));
    end
    aux(11) = aux(11) + 1;
end

% ── 현재 시각으로 전파 + 안전장치
n = aux(2);
tnow = (n-1)*dt;                      % 창 시작 기준 현재 시각
out = zeros(3,1);
fault = 0;
for k = 1:3
    o = (k-1)*nb;
    p0 = st((k-1)*3+1); v0 = st((k-1)*3+2); b = st((k-1)*3+3);
    idx = mod(head - n + (0:n-1), nb) + 1;
    S = 0; vv = 0;
    for j = 1:n-1
        S  = S + vv*dt + 0.5*acc(o+idx(j))*dt*dt;
        vv = vv + acc(o+idx(j))*dt;
    end
    out(k) = p0 + v0*tnow + 0.5*b*tnow*tnow + S;
end

if aux(10) == 1
    % ① 드롭아웃 — 측정이 끊기면 추측항법을 멈추고 마지막 측정에 눌러앉는다.
    %    없으면 VIO 가 죽어도 혼자 적분하며 그럴듯한 거짓말을 먹인다 = 최악 실패모드.
    if aux(5) > cfg.dropout
        out = aux(6:8); fault = 1;
    else
        % ② 혁신 게이트 — 예측과 측정이 말이 안 되게 벌어지면 센서로 스냅.
        %    센서가 닻이고 모델은 사이를 메우는 도구다.
        if norm(out - aux(6:8)) > cfg.innov_max
            out = aux(6:8); fault = 2;
            st = zeros(9,1);
            for k = 1:3; st((k-1)*3+1) = aux(5+k); end
        else
            % ③ 이탈 한계 — 물리적으로 불가능한 거리만큼 못 가게 자른다.
            %    ★ 경과는 (마지막 도착 후) + (그 측정의 나이) 다. 도착 후만 쓰면
            %      한계가 나이만큼 짧아져 정상 예측까지 자른다 — 파이썬 판에서
            %      2499 스텝 중 2478 번 잘렸던 버그가 그것이다 (08-29).
            elapsed = aux(5) + aux(9);
            lim = cfg.v_max*elapsed + cfg.dev_margin;
            dev = out - aux(6:8);
            if norm(dev) > lim
                out = aux(6:8) + dev * (lim / norm(dev)); fault = 3;
            end
        end
    end
end
aux(12) = fault;

block.Dwork(1).Data = acc;  block.Dwork(2).Data = meas;
block.Dwork(3).Data = mv;   block.Dwork(4).Data = st;
block.Dwork(5).Data = aux;  block.Dwork(6).Data = out;
end

function c = mhe_cfg()
%MHE_CFG  base 워크스페이스에서 설정을 읽는다 (qc_mhe_defaults 가 채운다).
gv = @(n,d) getb(n,d);
c.g          = gv('mhe_g',          9.80665);
c.horizon    = gv('mhe_horizon_s',  0.25);
c.meas_age   = gv('dly_pos_s',      0.0);    % 보상할 지연 = 위치 경로 지연
c.meas_std   = gv('mhe_meas_std',   0.01);
c.bias_rw    = gv('mhe_bias_rw',    3.0);
c.prior_p    = gv('mhe_prior_p_std', 1.0);
c.prior_v    = gv('mhe_prior_v_std', 1.0);
c.tol        = gv('mhe_tol_m',      0.005);
c.age_trust  = gv('mhe_age_trust',  0.7);
c.dropout    = gv('mhe_dropout_s',  0.25);
c.innov_max  = gv('mhe_innov_max',  0.50);
c.v_max      = gv('mhe_v_max',      3.0);
c.dev_margin = gv('mhe_dev_margin', 0.05);
c.n_max      = gv('mhe_n_max',      200);
end

function v = getb(name, dflt)
% 중첩 따옴표는 생성 과정에서 잘 깨진다 (실제로 깨져서 MATLAB 의 var 함수가
% 불렸다 — 08-29). char(39) 로 조립하면 그 위험이 없다.
Q = char(39);
if evalin('base', ['exist(' Q name Q ',' Q 'var' Q ')'])
    v = evalin('base', name);
else
    v = dflt;
end
end
