# robonex-common

RoboNex 저장소들이 함께 사용하는 로봇 관절, 모터, CAN 프로토콜, 정책 계약 패키지입니다.

이 저장소에는 형상·메시·URDF·USD·MJCF와 특정 정책의 가중치 또는 action 정규화 값이 들어가지 않습니다. 형상은 `robonex_description`, 정책별 계약은 각 정책의 `policy_manifest.json`이 소유합니다.

## 설치

```bash
git clone https://github.com/Humanoid-Project/robonex-common.git
python -m pip install -e ./robonex-common
```

CAN 런타임까지 필요한 경우:

```bash
python -m pip install -e './robonex-common[can]'
```

## 범위

- 12개 구동 관절의 ID, CAN 채널, 모델 이름, 모터 종류
- 구동 관절의 기계적 한계와 폐루프 수동 관절 목록
- RS02/RS03 통신 스펙과 RobStride CAN 프로토콜 상수
- CAN 메시지 변환, 모터 인터페이스, 단일 수신자 피드백 라우팅
- 정책 manifest 읽기·검증·저장

폐루프 수동 관절은 모델 상태 확인용 목록일 뿐 CAN 또는 정책 제어 대상이 아닙니다.
