# 단차 경계와 보행 폭을 고려한 Grid soft cost 시험

`grid_cost_experiment_sobel_footprint.yaml`을 `planner_params`로 선택한다.
기존 `grid_cost_experiment.yaml` 및 launch 기본값은 변경하지 않았다.
두 새 C++ parameter의 기본값은 0이므로 이전 point-Sobel 계산을 보존한다.

## 형상과 마진의 근거

참조 URDF: `/home/kim/rubi_ws/src/rubi_gazebo_sim/models/RUBI/urdf/RUBI.urdf`.
BODY collision을 BODY 좌표계 XY에 투영한 외접반경은 0.264935 m다.
TIP collision은 반경 0.032 m의 구이며, 현재 실행 world의 SDF에서도 동일하다.
기존 두 navigation bag에서 BODY 기준 TIP frame 원점의 최대 XY 반경은
0.247610 m였다. 구의 크기까지 합하면 0.279610 m이므로 반경을 0.28 m로 올림했다.
사용자 요청인 로봇 반경 + 5~10 cm에서 마진은 우선 0.10 m를 사용한다.

- `evaluation.grid_sobel_kernel_size: 5`
- `evaluation.grid_sobel_gradient_cost_weight: 2.0`
- `evaluation.grid_sobel_footprint_radius_m: 0.28`
- `evaluation.grid_sobel_footprint_falloff_m: 0.10`
- 기존 Local Relief 0.10 m hard reject 유지
- Sobel hard reject 및 별도 terrain hard clearance는 비활성 유지

현재 Gazebo SDF는 원본 URDF BODY collision 중 일부만 유지한다. 원본 BODY 형상은
보수적인 로봇 크기 참고값으로 사용했다. 보행 반경은 두 과거 기록을 근거로 한
실험값이며, 모든 자세/속도/미래 보행의 도달 가능 영역을 보장하지 않는다.
BODY 기울기, 발의 세계 좌표 투영, 현재 추종·정합 오차까지 인증한 값도 아니다.

## 계산 방식

기존 5×5 Sobel 응답을 G(c), 셀 중심 간 거리를 d라고 할 때:

```
k(d) = 1                         if d <= 0.28 m
       (0.38 - d) / 0.10         if 0.28 < d < 0.38 m
       0                         otherwise
G_envelope(c) = max_q k(distance(c,q)) * G(q)
C_sobel(edge) = w_s * edge_length * mean_endpoint(G_envelope)
```

0.38 m는 Sobel 응답을 확장하는 지지 범위이다. 5×5 필터와 지도 번짐 자체의 폭이
더해지므로 물리적 단차 모서리에서 정확히 0.38 m 떨어진 영역과 동일하지 않다.
확장된 항은 측정된 mechanical energy/CoT가 아닌 유한한 기하학적 접근 비용이다.
주변 셀을 모두 합산하지 않고 max를 사용하여 이웃 샘플 수에 따른 중복 합산을 피한다.
상승 비용식은 그대로이며, soft cost 때문에 통과 가능한 단차를 새로 hard reject하지 않는다.

A* 인접 이동, 연속 경로 재평가, smoothing/refinement 검증이 같은 확장 비용장을 사용한다.
기존 `sobel_cost`, `sobel_gradient_exposure_m`은 이 옵션을 켜면 확장된 비용장을 의미한다.
`max_grid_sobel_gradient`도 해당 비용장 값이고, 원래 경계 gradient만을 뜻하지 않는다.
`plan_metrics.parameters`에는 반경과 마진이 함께 기록된다.
Unknown gradient를 새로 0으로 채우지 않는다. 유효한 이웃 gradient만 사용하고,
기존 costmap/height evidence 차단 규칙을 유지한다. Footprint 전체의 unknown 공간을
별도로 충돌 검사하는 기능은 아니며, soft cost는 최소 clearance 보장이 아니다.

## 실행과 비교

Vanilla와 CoT에 동일한 새 YAML을 지정한다. 처음에는 기존 Vanilla height_weight=5를
유지하여 경계 처리 변화부터 확인한다. CoT weight=1은 별도 pilot로 기록한다.
실험 navigation을 재시작해야 새 코드/설정이 적용된다. 새 run_id와 bag 경로를 사용한다.
시뮬레이터 및 localization은 이번 기능을 위해 재설정할 필요가 없다.

다음 항목을 구분해 확인한다:
1. raw global path와 최종 tracking path가 단차 경계에 얼마나 접근하는가.
2. 실제 BODY 및 발 궤적, tracking 오차, 단차 접촉 여부.
3. 실제 work/CoT, 도착 성공, 주행 거리와 시간.

현재 world에는 contact_frame/bumper 관측 토픽이 없다. 이 기능만 적용해서
물리적 발 접촉 감소나 work 개선이 검증됐다고 결론 내리지 않는다.

## 검증 기록 (2026-11-16)

- 해당 workspace에서 Release 빌드 및 설치 완료.
- 새 C++ 6개 시나리오: 평지측 경계 접근 비용, 유한한 감쇠, 평지에서 0,
  통과 가능/불가능 단차의 기존 판정, 연속/인접 경로 일치, A* 우회 선택,
  잘못된 parameter 거절을 확인했다.
- 관련 기존 C++ 7개 test suite도 통과했다.
- 새 YAML이 Sobel/반경 옵션 외의 비교 조건을 유지하는 검사 통과.
- DDS domain 210에서 controller 없이 실제 frozen-site launch 검증:
  기존 Vanilla 5, 새 Vanilla 5, 새 CoT 1 모두 accepted. /cmd_vel publisher 없음.
- 이전 config contract 2개는 HEAD에서도 이미 테스트 기대값과 불일치한다:
  hybrid_navigation_controller 주기(기대 20 Hz / 설정 10 Hz), historical
  hybrid_grid_cot_cost height weight(기대 1 / 설정 0). 이번 설정과 무관하며 변경하지 않았다.
- 실제 Gazebo 이동/접촉/에너지 개선은 아직 시험하지 않았다.

세부 근거와 검사 출력:
`/home/kim/Documents/ChatGPT/RCLab RUBI Height Map Based Navigation/output/sobel_footprint_pilot_20261116/`
