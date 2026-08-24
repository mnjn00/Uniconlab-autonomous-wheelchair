# base_model: 어느 사본이 도는가

`base_model` 은 휠체어 베이스와 시리얼로 말하는 팀 공용 패키지이고,
**어떤 git 저장소에도 들어 있지 않습니다.** NUC 에만 존재합니다.

## 실행본

    ~/catkin_ws/src/base_model          <- rospack find base_model 이 가리키는 곳
    ~/Documents/base_model              <- 오래된 사본, 실행되지 않음

`~/Documents` 쪽은 `ROS_PACKAGE_PATH` 에 없어서 실행을 가로채지 않지만,
파일 이름이 같아서 읽는 쪽이 헷갈립니다. 2026-08-20 에 실제로 그 착각으로
없는 결함을 보고한 적이 있습니다. 그쪽에는 표식 파일을 두었습니다.

## 노드 이름이 파일 이름과 다르다

`wheel.launch` 는 바퀴 명령 노드를 `wheel_cmd` 라는 이름으로 띄우지만
파일은 `wheel_cmd_tmp.py` 입니다. `wheel_cmd.py` 도 옆에 있고, 그건 안 돕니다.

    <node pkg="base_model" type="uart.py"          name="uart"      output="screen"/>
    <node pkg="base_model" type="odom_pub.py"      name="odom_pub"  output="screen"/>
    <node pkg="base_model" type="wheel_cmd_tmp.py" name="wheel_cmd" output="screen"/>

## wheel_cmd 기준 소스와 배포 상태

저장소에서 검토하고 테스트하는 기준 소스는
`tools/base_model_wheel_cmd_guard.py` 입니다. 이 파일을 커밋하는 것만으로 NUC의
실행본이 바뀌지는 않습니다. 현장 반영 때는 별도 배포 절차로 아래 파일에 복사하고
노드를 재시작한 뒤 두 파일의 SHA-256이 같은지 확인해야 합니다.

    repo: tools/base_model_wheel_cmd_guard.py
    NUC:  ~/catkin_ws/src/base_model/src/wheel_cmd_tmp.py

2026-08-24의 정지 램프·정지 감시·DWA 진단 작업은 사용자의 요청에 따라 로컬에서만
수행했습니다. NUC 파일 복사, ROS 노드 재시작, GitHub push는 하지 않았습니다.
따라서 이 로컬 커밋을 NUC에서 이미 실행 중인 것으로 해석하면 안 됩니다.

## 시리얼 프레임

    wheel_cmd     [ll, lv, rr, rv, brk]
    wheel_status  [72, mode, ll, lv, rr, rv, brk, battery, checksum, 13, 10]

방향 `C`(0x43) 전진, `W`(0x57) 후진, `S`(0x53) 정지.
속도 `(byte - 0x21) / 10 / 3.6` m/s. mode `A`(65) 자동, `M`(77) 수동.
`battery` 는 55/66/77/88/99 다섯 값만 갖는 막대 표시이지 전압이 아닙니다.

## 스냅샷

`base_model_uart.py` 는 실행본 `uart.py` 의 사본입니다. 버전 관리가 안 되는
파일이라 여기 남깁니다. 원본을 고쳤으면 이 사본도 같이 갱신하십시오.
