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

## 시리얼 프레임

    wheel_cmd     [ll, lv, rr, rv, brk]
    wheel_status  [72, mode, ll, lv, rr, rv, brk, battery, checksum, 13, 10]

방향 `C`(0x43) 전진, `W`(0x57) 후진, `S`(0x53) 정지.
속도 `(byte - 0x21) / 10 / 3.6` m/s. mode `A`(65) 자동, `M`(77) 수동.
`battery` 는 55/66/77/88/99 다섯 값만 갖는 막대 표시이지 전압이 아닙니다.

## 스냅샷

`base_model_uart.py` 는 실행본 `uart.py` 의 사본입니다. 버전 관리가 안 되는
파일이라 여기 남깁니다. 원본을 고쳤으면 이 사본도 같이 갱신하십시오.
