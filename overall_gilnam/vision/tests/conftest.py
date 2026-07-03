# tests가 vision/ 모듈을 패키지 설치 없이 import할 수 있게 경로 추가
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
