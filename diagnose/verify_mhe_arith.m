%% MHE S-function 의 Update 산술을 Simulink 없이 검사 (2026-08-29)
%%
%% Simulink 안에서 S-function 이 죽으면 MATLAB 이 통째로 크래시해 원인을 못 본다
%% (실제로 그렇게 죽었다). 그래서 링 버퍼·인덱스 산술만 떼어 여기서 훑는다 —
%% 배열 범위 밖 접근이 있으면 평범한 MATLAB 오류로 잡힌다.

modelDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(modelDir, 'Scripts_Data'));
clear functions

dt = 0.001; H = 0.25; nb = max(8, ceil(H/dt) + 1);
fprintf('링 크기 nb = %d, dt = %g s\n', nb, dt);

acc = zeros(3*nb,1); meas = zeros(3*nb,1); mv = zeros(3*nb,1);
st  = zeros(9,1); aux = zeros(12,1); aux(4) = 1;
bad = 0;

for step = 1:1500
    % --- 링 전진 (S-function 과 같은 산술)
    head = mod(aux(1), nb) + 1; aux(1) = head;
    if aux(2) >= nb
        oldest = mod(head - nb, nb) + 1;
        if oldest < 1 || oldest > nb; fprintf(2,'[%d] oldest=%d 범위 밖\n', step, oldest); bad = bad+1; end
        for k = 1:3
            p0 = st((k-1)*3+1); v0 = st((k-1)*3+2); b = st((k-1)*3+3);
            a0 = acc((k-1)*nb + oldest);
            st((k-1)*3+1) = p0 + v0*dt + 0.5*(b + a0)*dt*dt;
            st((k-1)*3+2) = v0 + (b + a0)*dt;
        end
    end
    aux(2) = min(aux(2) + 1, nb);
    for k = 1:3
        acc((k-1)*nb + head) = 0.3*sin(step/97);
        mv((k-1)*nb + head)  = 0;
    end

    % --- 측정 (30 Hz 흉내), age_trust 0.7, tau 60 ms
    aux(5) = aux(5) + dt;
    if mod(step, 33) == 0
        age = min(0.060, H) * 0.7;
        back = min(round(age/dt), aux(2)-1);
        slot = mod(head - 1 - back, nb) + 1;
        if slot < 1 || slot > nb; fprintf(2,'[%d] slot=%d 범위 밖\n', step, slot); bad = bad+1; end
        for k = 1:3
            meas((k-1)*nb + slot) = 0.01*step;
            mv((k-1)*nb + slot)   = 1;
        end
        aux(6:8) = [0.01*step; 0; 1]; aux(5) = 0; aux(10) = 1; aux(9) = back*dt;
    end

    % --- 닻 기반 출력 (새 구조)
    n = aux(2);
    if aux(10) == 1
        ageN = max(0, min(round(aux(9)/dt) + round(aux(5)/dt), n-1));
        idx = mod(head - n + (0:n-1), nb) + 1;
        if any(idx < 1) || any(idx > nb); fprintf(2,'[%d] idx 범위 밖\n', step); bad = bad+1; end
        for k = 1:3
            o = (k-1)*nb; b = st((k-1)*3+3);
            d0 = (n-1-ageN)*dt;
            vv = st((k-1)*3+2) + b*d0;
            for j = 1:(n-1-ageN)
                vv = vv + acc(o+idx(j))*dt;
            end
            S = 0;
            for j = (n-ageN):(n-1)
                S  = S + vv*dt + 0.5*(acc(o+idx(j)) + b)*dt*dt;
                vv = vv + (acc(o+idx(j)) + b)*dt;
            end
            outk = aux(5+k) + S;
            if ~isfinite(outk); fprintf(2,'[%d] 축%d 출력 비유한 (%g)\n', step, k, outk); bad = bad+1; end
        end
    end

    % --- 무거운 해 (N 스텝마다)
    aux(3) = aux(3) + 1;
    if aux(3) >= aux(4)
        aux(3) = 0;
        idx = mod(head - n + (0:n-1), nb) + 1;
        trel = (0:n-1)' * dt;
        w = [1; 1; 1/9; 1/(0.01^2)];
        for k = 1:3
            o = (k-1)*nb;
            [p0,v0,b,dbs,ok] = qc_mhe_solve(trel, acc(o+idx), meas(o+idx), ...
                                            mv(o+idx), st((k-1)*3+(1:3)), w);
            if ok
                st((k-1)*3+(1:3)) = [p0; v0; b];
                if ~all(isfinite([p0 v0 b])); fprintf(2,'[%d] 해 비유한\n', step); bad = bad+1; end
            end
        end
        if isfinite(dbs) && dbs > 0
            aux(4) = max(1, min(200, floor(sqrt(2*0.005/dbs)/dt)));
        end
    end
end

fprintf('1500 스텝 완주. 문제 %d 건\n', bad);
fprintf('  최종 N = %d, nfill = %d, 바이어스 = [%.4f %.4f %.4f]\n', ...
        aux(4), aux(2), st(3), st(6), st(9));
