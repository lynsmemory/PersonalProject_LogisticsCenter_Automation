# 물류 시뮬레이션 차량 프리팹

Unity 단위 m, +Y 위, +Z 전방. 제조사 CAD가 아닌 기본 형상 조합 모델입니다.

- WingBody5t_Coupang.prefab: 흰색 5톤 윙바디 형태. 적재함 내부 기준 6.2×2.3×2.4m, 바닥 상면 1.1m. 1100mm 파레트 2열×5자리. 양쪽 윙과 운전석에 쿠팡 표기. 5톤은 모델 분류이며 물리 하중 검증값이 아닙니다.
- RobotForklift.prefab: 차체·마스트·승강 캐리지·포크·센서 형상. ForkCarriage의 local Y를 0~1.1m로 변경해 포크 상면을 0.08~1.18m로 이동할 수 있습니다. 포크와 실제 파레트 포켓의 간섭 검증이 필요합니다.
- LeftWingPivot: local Z=-110도, RightWingPivot: local Z=110도로 열 수 있습니다. 닫힌 상태는 0도입니다. 후방 상차 시 RearDoor 피벗도 별도로 열어야 합니다.
- 로고는 사용자가 제공한 사진을 참고한 글꼴 기반 근사본입니다. 공식 벡터 로고와 정확히 일치하지 않습니다. Textures/CoupangSide.png를 교체할 수 있습니다. 양쪽 로고는 윙 자식이므로 개폐 시 따라갑니다.
- PalletSlot은 파레트 밑면 중앙 기준입니다. 도크·접근·항법·파레트 부착 기준점을 포함합니다.
- 단순 BoxCollider가 있습니다. 주행·장애물 감지·상차·윙 개폐 애니메이션 제어와 Rigidbody는 연결 전입니다.

Unity 배치 실행이 프로젝트 초기 로딩에서 종료되어 파일을 직접 생성했습니다. 로컬 참조와 파레트 슬롯 경계는 검사했으나 Unity 임포트·시각 확인·물리 시험은 미실시입니다. Unity에서 프리팹을 열어 확인하세요.

생성기: Docs/Research/build_vehicle_prefabs.py. Unity에서 Tools/Logistics/Create Vehicle Prefabs 메뉴로도 같은 계열의 프리팹을 재생성할 수 있습니다. 로고 텍스처는 별도 파일로 유지됩니다.
